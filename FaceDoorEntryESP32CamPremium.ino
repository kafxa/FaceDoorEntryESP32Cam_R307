#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "Adafruit_Fingerprint.h"

const char *ssid = "iQ ExpandtQ 2.4G";
const char *password = "Gigabyte@3.14";

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

using namespace websockets;
WebsocketsServer socket_server;

camera_fb_t *fb = NULL;

long current_millis;
long last_detected_millis = 0;

#define relay_pin 2 // pin 12 can also be used
unsigned long door_opened_millis = 0;
long interval = 5000; // open lock for ... milliseconds
bool face_recognised = false;
bool fingerprintRecognized = false;
void app_facenet_main();
void app_httpserver_init();

// Assign the fingerprint sensor to Serial2
HardwareSerial hs(2);
Adafruit_Fingerprint finger(&hs);

typedef struct
{
  uint8_t *rawImageData;
  box_array_t *detectedObjects;
  dl_matrix3d_t *faceIdentifier;
} ImageProcessingResult;

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

httpd_handle_t camera_httpd = NULL;

typedef enum {
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
  START_FINGERPRINT_ENROLL,
  FINGERPRINT_ENROLL_COMPLETE,
  START_FINGERPRINT_DETECT
} en_fsm_state;
en_fsm_state global_state;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;


void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  digitalWrite(relay_pin, LOW);
  pinMode(relay_pin, OUTPUT);

  hs.begin(57600, SERIAL_8N1, 13, 15); // RX = GPIO 12, TX = GPIO 13
  //finger.begin(57600);

  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x"));
  Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x"));
  Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: "));
  Serial.println(finger.capacity);
  Serial.print(F("Security level: "));
  Serial.println(finger.security_level);
  Serial.print(F("Device address: "));
  Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: "));
  Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: "));
  Serial.println(finger.baud_rate);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;


  if (psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  app_httpserver_init();
  app_facenet_main();
  socket_server.listen(82);

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

int getFreeId()
{
  int id;
  for (id = 1; id < 127; id++)
  {
    uint8_t p = finger.loadModel(id);
    if (p == FINGERPRINT_NOTFOUND)
    {
      return id; // Found a free ID
    }
  }
  return -1; // No free ID found
}

uint8_t enrollFingerprint(WebsocketsClient &client)
{
  int id = getFreeId(); // Find a free ID for the new fingerprint
  if (id < 0)
  {
    Serial.println("Failed to find free ID for fingerprint");
    return 0;
  }

  Serial.print("Enrolling ID #");
  Serial.println(id);
  client.send("Place your finger on the sensor");

  int p = -1;
  Serial.print("Waiting for valid finger to enroll as #");
  Serial.println(id);
  while (p != FINGERPRINT_OK)
  {
    p = finger.getImage();
    switch (p)
    {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print("No finger detected");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(1);
  switch (p)
  {
  case FINGERPRINT_OK:
    Serial.println("Image converted");
    break;
  case FINGERPRINT_IMAGEMESS:
    Serial.println("Image too messy");
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    Serial.println("Communication error");
    return p;
  case FINGERPRINT_FEATUREFAIL:
    Serial.println("Could not find fingerprint features");
    return p;
  case FINGERPRINT_INVALIDIMAGE:
    Serial.println("Could not find fingerprint features");
    return p;
  default:
    Serial.println("Unknown error");
    return p;
  }

  Serial.println("Remove finger");
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER)
  {
    p = finger.getImage();
  }
  Serial.print("ID ");
  Serial.println(id);
  p = -1;
  Serial.println("Place same finger again");
  while (p != FINGERPRINT_OK)
  {
    p = finger.getImage();
    switch (p)
    {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(2);
  switch (p)
  {
  case FINGERPRINT_OK:
    Serial.println("Image converted");
    break;
  case FINGERPRINT_IMAGEMESS:
    Serial.println("Image too messy");
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    Serial.println("Communication error");
    return p;
  case FINGERPRINT_FEATUREFAIL:
    Serial.println("Could not find fingerprint features");
    return p;
  case FINGERPRINT_INVALIDIMAGE:
    Serial.println("Could not find fingerprint features");
    return p;
  default:
    Serial.println("Unknown error");
    return p;
  }

  // OK converted!
  Serial.print("Creating model for #");
  Serial.println(id);

  p = finger.createModel();
  if (p == FINGERPRINT_OK)
  {
    Serial.println("Prints matched!");
  }
  else if (p == FINGERPRINT_PACKETRECIEVEERR)
  {
    Serial.println("Communication error");
    return p;
  }
  else if (p == FINGERPRINT_ENROLLMISMATCH)
  {
    Serial.println("Fingerprints did not match");
    return p;
  }
  else
  {
    Serial.println("Unknown error");
    return p;
  }

  Serial.print("ID ");
  Serial.println(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK)
  {
    Serial.println("Stored!");
  }
  else if (p == FINGERPRINT_PACKETRECIEVEERR)
  {
    Serial.println("Communication error");
    return p;
  }
  else if (p == FINGERPRINT_BADLOCATION)
  {
    Serial.println("Could not store in that location");
    return p;
  }
  else if (p == FINGERPRINT_FLASHERR)
  {
    Serial.println("Error writing to flash");
    return p;
  }
  else
  {
    Serial.println("Unknown error");
    return p;
  }
  Serial.println("Fingerprint enrolled successfully!");
  return true;
}

uint16_t detectFingerprint()
{
  uint8_t p = finger.getImage();
  switch (p)
  {
  case FINGERPRINT_OK:
    Serial.println("Image taken");
    break;
  case FINGERPRINT_NOFINGER:
    Serial.println("No finger detected");
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    Serial.println("Communication error");
    return p;
  case FINGERPRINT_IMAGEFAIL:
    Serial.println("Imaging error");
    return p;
  default:
    Serial.println("Unknown error");
    return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p)
  {
  case FINGERPRINT_OK:
    Serial.println("Image converted");
    break;
  case FINGERPRINT_IMAGEMESS:
    Serial.println("Image too messy");
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    Serial.println("Communication error");
    return p;
  case FINGERPRINT_FEATUREFAIL:
    Serial.println("Could not find fingerprint features");
    return p;
  case FINGERPRINT_INVALIDIMAGE:
    Serial.println("Could not find fingerprint features");
    return p;
  default:
    Serial.println("Unknown error");
    return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK)
  {
    Serial.println("Found a print match!");
  }
  else if (p == FINGERPRINT_PACKETRECIEVEERR)
  {
    Serial.println("Communication error");
    return p;
  }
  else if (p == FINGERPRINT_NOTFOUND)
  {
    Serial.println("Did not find a match");
    return p;
  }
  else
  {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #");
  Serial.print(finger.fingerID);
  Serial.print(" with confidence of ");
  Serial.println(finger.confidence);

  return 1;
}

static esp_err_t index_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_handler,
  .user_ctx  = NULL
};

void app_httpserver_init()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
    Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}

void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

static esp_err_t send_face_list(WebsocketsClient &client)
{
  client.send("delete_faces"); // tell browser to delete all faces
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) // loop current faces
  {
    sprintf(add_face, "listface:%s", head->id_name);
    client.send(add_face); // send face to browser
    head = head->next;
  }
return ESP_OK;
}

static esp_err_t delete_all_faces(WebsocketsClient &client)
{
  delete_face_all_in_flash_with_name(&st_face_list);
  client.send("delete_faces");
return ESP_OK;
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg)
{
  if (msg.data() == "stream")
  {
    global_state = START_STREAM;
    client.send("STREAMING");
  }
  if (msg.data() == "detect")
  {
    global_state = START_DETECT;
    client.send("DETECTING");
  }
  if (msg.data().substring(0, 8) == "capture:")
  {
    global_state = START_ENROLL;
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {
        0,
    };
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1);
    client.send("CAPTURING");
  }
  if (msg.data() == "recognise")
  {
    global_state = START_RECOGNITION;
    client.send("RECOGNISING");
  }
  if (msg.data().substring(0, 7) == "remove:")
  {
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));
    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client); // reset faces in the browser
  }
  if (msg.data() == "delete_all")
  {
    delete_all_faces(client);
  }
if (msg.data() == ("store_fingerprint:"))
  {
    String personName = msg.data().substring(18); // Extract person's name from the message
    Serial.println("Storing fingerprint for: " + personName);
    // Call function to start fingerprint enrollment process
    enrollFingerprint(client);
  }
  if (msg.data() == ("detect_fingerprint"))
  {
    String personName = msg.data().substring(18);
    global_state = START_FINGERPRINT_DETECT;
    if (enrollFingerprint(client))
    { 
      client.send("fingerprint_recognized");
      fingerprintRecognized = true; // Assume this is a global or appropriately scoped variable
    }
    else
    {
      client.send("fingerprint_not_recognized");
    }
  }
}

void open_door(WebsocketsClient &client)
{
  if (digitalRead(relay_pin) == LOW)
  {
    digitalWrite(relay_pin, HIGH); // close (energise) relay so door unlocks
    Serial.println("Door Unlocked");
    client.send("door_open");
    door_opened_millis = millis(); // time relay closed and door opened
  }
}
void handleFingerprintDetected(WebsocketsClient &client)
{
  // Log the successful fingerprint detection
  Serial.println("Fingerprint detected and verified.");
  Serial.print("ID: ");
  Serial.print(finger.fingerID);
  Serial.print(", Confidence: ");
  Serial.println(finger.confidence);

  // Send a message to the connected web client
  client.send("Fingerprint recognized.");

  // Update global state to reflect the fingerprint was recognized
  fingerprintRecognized = true;
  }


void loop()
{
  // Accept a new client connection request on the server socket
  auto client = socket_server.accept();
// Set up a message handler for the client connection
  // The 'handle_message' function will be called whenever the client sends a message to the server
  client.onMessage(handle_message);
// Allocate memory for a 3D matrix to store image data
  // The parameters '1, 320, 240, 3' represent the number of matrices, the width, the height, and the number of color channels (3 for RGB), respectively
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  // Initialize a structure to store the result of the image processing operation
  ImageProcessingResult processedImageResult = {0};
  // Assign the pointer to the allocated image data to the 'image' field of the 'processedImageResult' structure
  // The image data will be stored in the 'processedImageResult' structure for further processing
  processedImageResult.rawImageData = image_matrix->item;

  send_face_list(client);
  client.send("STREAMING");

  enrollFingerprint(client);
  
  // Main loop that runs continuously after setup
  while (client.available()) {
// Process any messages from the client
    client.poll();

    // If the door has been open longer than the specified interval, lock it
  if (millis() - interval > door_opened_millis) {
      digitalWrite(relay_pin, LOW); // Lock the door
    }

    // Attempt to capture a camera frame
    fb = esp_camera_fb_get();

    // Flags to track detection status
    bool faceDetected = false;
    bool fingerprintDetected = false;
  if (fb && (global_state == START_DETECT || global_state == START_ENROLL || global_state == START_RECOGNITION || global_state == START_FINGERPRINT_DETECT || global_state == START_FINGERPRINT_ENROLL)) {
    // تحضر ل ايجاد الوجوه و البصمات
    processedImageResult.detectedObjects = NULL;
    processedImageResult.faceIdentifier = NULL;

    // تحوير نوع الصورة الى RGB
    fmt2rgb888(fb->buf, fb->len, fb->format, processedImageResult.rawImageData);
    // ايجاد الوجوه
    processedImageResult.detectedObjects = face_detect(image_matrix, &mtmn_config);

    if (processedImageResult.detectedObjects)
    {
        if (align_face(processedImageResult.detectedObjects, image_matrix, aligned_face) == ESP_OK)
        {

          processedImageResult.faceIdentifier = get_face_id(aligned_face);
          last_detected_millis = millis();
          if (global_state == START_DETECT) {
            client.send("FACE DETECTED");
          }

          if (global_state == START_ENROLL)
          {
            int left_sample_face = do_enrollment(&st_face_list, processedImageResult.faceIdentifier);
            char enrolling_message[64];
            sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
            client.send(enrolling_message);
            if (left_sample_face == 0)
            {
              ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
              global_state = START_STREAM;
              char captured_message[64];
              sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
              client.send(captured_message);
              send_face_list(client);

            }
          }
          if (global_state == START_FINGERPRINT_DETECT || global_state == START_FINGERPRINT_ENROLL)
          {
            if (!detectFingerprint()==0)
            {  
              fingerprintDetected = true;
              handleFingerprintDetected(client); // Handle successful fingerprint detection
              global_state = ENROLL_COMPLETE;
          }}
          if (global_state == START_RECOGNITION &&  global_state == ENROLL_COMPLETE  && (st_face_list.count > 0))
          {
            face_id_node *f = recognize_face_with_name(&st_face_list, processedImageResult.faceIdentifier);
            if (f)
            {
              char recognised_message[64];
              sprintf(recognised_message, "DOOR OPEN FOR %s", f->id_name);
              open_door(client);
              client.send(recognised_message);
            }
            else
            {
              client.send("FACE NOT RECOGNISED");
            }
          }
          dl_matrix3d_free(processedImageResult.faceIdentifier);
        }

      }
      else
      {
        if (global_state != START_DETECT) {
          client.send("NO FACE DETECTED");
        }
      }

      if (global_state == START_DETECT && millis() - last_detected_millis > 500) { // Detecting but no face detected
        client.send("DETECTING");
      }

    }

    client.sendBinary((const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    fb = NULL;
  }
}
