#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

//#include <libserial/SerialPort.h>
#include <serial/serial.h>
#include <iostream>
#include <math.h>

using namespace  serial;

serial::Serial pSerial;

bool leer_datos(uint8_t Servo, uint8_t Direc_Memo, uint8_t orden=2, uint8_t param=2, int posi_env=0) {
  int chec_sum = 0;
  std::string cadena;  
  pSerial.flush ();
  std::vector<uint8_t> buf_l(8);
  buf_l[0] = 0xFF;
  buf_l[1] = 0xFF;
  buf_l[2] = Servo; //ID
  buf_l[3] = 0x04; // Longitud
  buf_l[4] = orden; // Orden lectura
  buf_l[5] = Direc_Memo; // Dirección principal del segmento de datos leído, si "posicion" = 0x38 ó 56 en binario
  buf_l[6] = param; // Longitud datos leidos
  
  for (int i = 2; i < 7; i++) {
     chec_sum += buf_l[i];
  }
  if (chec_sum > 255){
     chec_sum = chec_sum - 256; ///////////////////// OJO 256
  }
  chec_sum = 0xFF - chec_sum;
  
  buf_l[7] = chec_sum; // Chec_sum byte de control
  printf("\n %d", chec_sum);
  
  if (orden == 2) cadena = "Lectura";
   else cadena = "Escritur";
  printf("\n ENVIO %s - %d %d %d %d %d %d %d %d", cadena.c_str(), buf_l[0], buf_l[1], buf_l[2], buf_l[3], buf_l[4], buf_l[5], buf_l[6], buf_l[7]);
  
  try {
      pSerial.write(buf_l);
  }
  catch (...) {
      printf("\n NO ESCRIBE");
  }
  if (Direc_Memo != 56) {
      uint8_t Dat_lei[8];
      size_t k = 8;
      pSerial.read(Dat_lei, k);
      printf("\n RECIBO %s - %d %d %d %d %d %d %d %d", cadena.c_str(), Dat_lei[0], Dat_lei[1], Dat_lei[2], Dat_lei[3], Dat_lei[4], Dat_lei[5], Dat_lei[6], Dat_lei[7]);
      return false;
      }
      else {
      uint8_t Dat_lei[12];
      size_t k = 12;
      int posi_leida;
      pSerial.read(Dat_lei, k);
      posi_leida = Dat_lei[5];
      posi_leida<<=8;
      posi_leida |= Dat_lei[6];
      printf("\n SERVO-%d : posi_leida %d, velocidad %d %d, PWM %d %d", Dat_lei[2], posi_leida, Dat_lei[7], Dat_lei[8], Dat_lei[9], Dat_lei[10]);
      if (abs(posi_env - posi_leida) < 10) return true;
      else return false;
      }
}

void mover(uint8_t Servo, int pos) {
  int chec_sum = 0;
  pSerial.flush ();
  std::vector<uint8_t> buf_p(13);
    buf_p[0] = 0xFF;
    buf_p[1] = 0xFF;
    buf_p[2] = Servo; //ID
    buf_p[3] = 0x09; // Longitud
    buf_p[4] = 0x04; // Orden escribir
    buf_p[5] = 0x2A; // Dirección principal del segmento de datos escrito "posicion" 
    buf_p[6] = (pos>>8); // Byte alto (pos>>8)
    buf_p[7] = (pos&0xff); // Byte bajo - 0800 (2048) "posicion" (pos&0xff)
    buf_p[8] = 0x00; // "tiempo"
    buf_p[9] = 0x00; //
    buf_p[10] = 0x00; // Byte alto
    buf_p[11] = 0x64; // Byte bajo - 03E8 (1000) "velocidad" // 0064 (100 velocida en decimal)

  for (int i = 2; i < 12; i++) {
     chec_sum += buf_p[i];
  }
  if (chec_sum > 255){
     chec_sum = chec_sum - 256; ///////////////////// OJO 256
  }
  chec_sum = 255 - chec_sum;
  buf_p[12] = chec_sum; // Chec summ
  printf("%d", chec_sum);
  
  printf("\n ENVIO MOVER - %d %d %d %d %d %d %d %d %d %d %d %d %d", buf_p[0], buf_p[1], buf_p[2], buf_p[3], buf_p[4], buf_p[5], buf_p[6], buf_p[7], buf_p[8], buf_p[9], buf_p[10], buf_p[11], buf_p[12]);
  
  try {
      pSerial.write(buf_p);
  }
  catch (...) {
      printf("\n NO ESCRIBE");
  }
    
  uint8_t Dat_lei[6];
  size_t k = 6;
    
  try {  
       pSerial.read(Dat_lei, k);
    }
    catch (...) {
       printf("\n NO LEE respuesta MOVER");
  }

  printf("\n DEV. MOVER - %d %d %d %d %d %d", Dat_lei[0], Dat_lei[1], Dat_lei[2], Dat_lei[3], Dat_lei[4], Dat_lei[5]);
}

  
int main(int argc, char** argv)
{
  // Set up ROS.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("arips_arm_node");

  
  //rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr brazo_pub = node->create_publisher<geometry_msgs::msg::Pose>("tema_pose", 1);
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr brazo_pub = node->create_publisher<std_msgs::msg::Int32>("pose", 1);
  
  pSerial.setPort("/dev/ttyUSB0");  
   
  pSerial.setBaudrate(1000000);
  pSerial.setBytesize(eightbits);
  pSerial.setParity(parity_none);
  pSerial.setStopbits(stopbits_one);
  pSerial.setTimeout(1, 0, 1, 0, 1);
  pSerial.open();

  
  if (pSerial.isOpen()) printf("\n CONECTADO ");
     else printf("\n NO conectado");
  
     
  int posicion1, posicion2;
  leer_datos(1, 26, 3, 1);
  leer_datos(1, 27, 3, 0);
  
  
  while (rclcpp::ok()) {
//    rclcpp::spin(node);
    std::cout<<"\n INTRODUCE SERVO 1: ";
    std::cin>> posicion1;
    std::cout<<"\n INTRODUCE SERVO 2: ";
    std::cin>> posicion2;
    mover(1, posicion1);
    mover(2, posicion2);
    int resultado = 0;
    while( resultado < 2 ) {
       if (leer_datos(1, 56, 2, 6, posicion1)) resultado += 1;
       if (leer_datos(2, 56, 2, 6, posicion2)) resultado += 1;
    }
  }
  pSerial.close();
}
