# Esp32 Cam Telegram

### Librerias
<img src="https://github.com/IDiegoUlises/Esp32-Cam-Telegram/blob/main/Imagenes/Universal-TelegramBot-Libreria.png" />

* Universal Telegram Bot

<img src="https://github.com/IDiegoUlises/Esp32-Cam-Telegram/blob/main/Imagenes/ArduinoJson-Libreria.png" />

* ArduinoJson se utiliza porque es una libreria de dependencia para utilizar la libreria Telegram Bot

### Crear Bot
<img src="https://github.com/IDiegoUlises/Esp32-Cam-Telegram/blob/main/Imagenes/IMG-20230911-WA0001.jpg" width="450" height="800" />

* Se debe obtener el token del Bot

### ID De Usuario
<img src="https://github.com/IDiegoUlises/Esp32-Cam-Telegram/blob/main/Imagenes/IMG-20230911-WA0000.jpg" width="450" height="800" />

* Se debe obtener el ID del usuario

### Codigo
```c++
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include "esp_camera.h"

// Wifi Credenciales
#define WIFI_SSID "Wifi Home"
#define WIFI_PASSWORD "S4m4sw3n0s"

//Telegram Bot Token
String BOTtoken = "6651295482:AAHSOXNTzMyJmrj6nuQi7wskSMFatI8Uyks";

//Identificador del usuario que puede utilizar el bot
String CHAT_ID  = "6615998413";

//Objeto WifiSecure y UniversalTelegramBot
WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

//Declaracion de pins para camara MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

//Pin para controlar el Led
int led = 2;

//Funcion para enviar la imagen de la camara
String sendPhotoTelegram()
{
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }

  Serial.println("Connect to " + String(myDomain));

  if (clientTCP.connect(myDomain, 443))
  {
    Serial.println("Connection successful");

    String head = "--TelegramBot\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--TelegramBot\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--TelegramBot--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;

    clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=TelegramBot");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        clientTCP.write(fbBuf, remainder);
      }
    }

    clientTCP.print(tail);

    esp_camera_fb_return(fb);

    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + waitTime) > millis()) {
      Serial.print(".");
      delay(100);
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state == true) getBody += String(c);
        if (c == '\n') {
          if (getAll.length() == 0) state = true;
          getAll = "";
        }
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length() > 0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}

void setup()
{
  //Inicia el puerto serial
  Serial.begin(115200);

  //Se conecta la a red wifi, en caso de no conectarse quedara en un bucle infinito
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); //Agrega el certificado
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  //Imprime la direccion IP
  Serial.print("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  //Configuracion de la camara
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

  //Verifica si es compatible con PSRAM y elige la configuracion adecuada
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_VGA; //Tamaño de la foto, se pueden elegir QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA, mientras mas pequeña mejor resolucion
    config.jpeg_quality = 10; //10-63 mientras menor el numero mejor calidad
    config.fb_count = 2;
  }

  else
  {
    config.frame_size = FRAMESIZE_VGA; //Tamaño de la foto, se pueden elegir QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA, mientras mas pequeña mejor resolucion
    config.jpeg_quality = 10; //10-63 mientras menor el numero mejor calidad
    config.fb_count = 2;
  }

  //Inicializar la camera con la configuracion
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.println("Camera no inicializada" + err);
    return;
  }

}

void loop()
{

  //Obtiene el numero de los mensajes recibidos
  int newMensaje = bot.getUpdates(bot.last_message_received + 1);

  //Bucle que recorre los mensajes
  for (int i = 0; i < newMensaje; i++)
  {
    //Obtiene el CHAT_ID del usuario de telegram
    String chat_id = String(bot.messages[i].chat_id);

    //Obtiene el mensaje que el usuario envia
    String text = bot.messages[i].text;

    //Obtiene el nombre de usuario
    String from_name = bot.messages[i].from_name;

    //Inicia el bucle cuando se recibe un nuevo mensaje
    while (newMensaje)
    {
      //Solo se ejecuta un solo if de todos por eso el primer if es la verificacion de usuario
      //no permite el uso de comandos si el usuario es diferente
      if (chat_id != CHAT_ID )
      {
        bot.sendMessage(chat_id, "No autorizado", "");
        Serial.println("No permitido");
      }

      //Si el usuario envia el comando /on se ejecutara la siguiente accion
      else if (text == "/on")
      {
        digitalWrite(led, HIGH);
        bot.sendMessage(chat_id, "Led Encendido", "");
        Serial.println("Led Encendido");
      }

      //Si el usuario envia el comando /off se ejecutara la siguiente accion
      else if (text == "/off")
      {
        digitalWrite(led, LOW);
        bot.sendMessage(chat_id, "Led Apagado", "");
        Serial.println("Led Encendido");
      }

      //Si el usuario envia el comando /photo se enviara una foto desde la camara
      else if (text == "/photo")
      {
        bot.sendMessage(chat_id, "Enviando foto", "");
        sendPhotoTelegram();
        Serial.println("Foto Enviada");

      }

      //En caso que no exista el comando
      else
      {
        bot.sendMessage(chat_id, "Comando no disponible", "");
        Serial.println("Comando no disponible");
      }

      //Actualiza el valor de nuevos mensajes es importante para salir del bucle while
      newMensaje = bot.getUpdates(bot.last_message_received + 1);
    }

  }

  //Espera la mitad de un segundo
  delay(500);

}
```

* En el dispositivo Esp32 Cam no existe una funcion para rotar 90 grados una captura de la camara

