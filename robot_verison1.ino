// 1ª version del robot autoequilibrado con sensor MPU 6050 
// Autor Andrés Vicente Arévalo 
// andreu_jaja96@hotmail.com


#include <Wire.h>

// (SDA - A4 )(SCL - A5)
#define ciclos_calibrado 3000
#define v_max 255
#define r_min 25

#define gx_ang_inicial 1.4
#define gy_ang_inicial -2.85
#define gz_ang_inicial -4.05

#define constante_rango_accel 4096.0
#define constante_rango_gyro 15

#define motor_dch_alante 10
#define motor_dch_atras  9
#define motor_izq_alante 6
#define motor_izq_atras 5
  
#define Kp 7
#define Ki 20
#define Kd 0.20
#define tiempo_ciclo  0.00305

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ,aux;

long gyroX, gyroZ;
float rotX, rotZ;

float gx_ang,gy_ang,gz_ang,actotal;
//float gx_ang_inicial,gy_ang_inicial,gz_ang_inicial;

float x_cal,y_cal,z_cal;                                          // definimos variables de calibrado y el numero de ciclos de este
float x_ang, z_ang;

float angulo_x_deseado, angulo_z_deseado;

float error_actual_x, error_actual_z;
float error_int_x , error_int_z;
float error_pasado_x, error_pasado_z;


float rp_x,rp_y,rp_z;
float ri_x,ri_y,ri_z;
float rd_x,rd_y,rd_z;
float r_x,r_y,r_z,r_izq,r_dch;

void setup() {
  
  Serial.begin(115200);
  Wire.begin();

  pinMode( motor_dch_alante ,OUTPUT ); 
  pinMode( motor_dch_atras , OUTPUT ); 
  pinMode( motor_izq_alante , OUTPUT );
  pinMode( motor_izq_atras , OUTPUT );
  
  setupMPU();
  Serial.println(' '); Serial.println(' ');
  Serial.println("Calibrando los valores del sistema ....");
  calibrar_valores();

  //inicializamos las variables del PID
  angulo_x_deseado = 0 ; angulo_z_deseado = 0 ;
  error_pasado_x = 0; error_pasado_z = 0;
  error_int_x = 0;  error_int_z = 0;
  
}

void loop() {
  
  recordregisters();
  processData();
  calculo_PID();
  correccion_posicion();
  visualizar_datos();
  
}


void setupMPU(){
  
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                    //End the transmission with the gyro.
  
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                                    //End the transmission with the gyro
  
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();                                                    //End the transmission with the gyro


  Wire.beginTransmission(0b1101000);                                      //Start communication with the address found during search
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                    //End the transmission with the gyro    
}

void recordregisters() {

  Wire.beginTransmission(0b1101000);          //I2C address of the MPU
  Wire.write(0x3B);                           //Starting register for Accel Readings
  Wire.endTransmission();
  
  Wire.requestFrom(0b1101000,6);              //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  
  accelX = Wire.read()<<8|Wire.read();        //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read();        //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read();        //Store last two bytes into accelZ
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  aux = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  
}


void calibrar_valores() 
{
  
  for ( int i = 0 ; i <= ciclos_calibrado ; i++ )
  {
    recordregisters();
    
    rotX = gyroX / constante_rango_gyro;
    rotZ = gyroZ / constante_rango_gyro;
    
    x_cal += rotX;
    z_cal += rotZ;

    //gForceX = accelX / constante_rango_accel ;
    //gForceZ = accelZ / constante_rango_accel ;

    //gx_ang_inicial += gForceX ;
    //gz_ang_inicial += gForceZ ;
    
  }
  
  Serial.println(".......");
  delay(500);
  
  //gx_ang_inicial = gx_ang_inicial / ciclos_calibrado ;
  //gz_ang_inicial = gz_ang_inicial / ciclos_calibrado ;
  
  x_cal = x_cal / ciclos_calibrado ;
  z_cal = z_cal / ciclos_calibrado ;
}


  
void processData(){                                                       //procesamos los datos para obtener el angulo en grados y la acceleración en g y 
  
  gForceX = accelX / constante_rango_accel; 
  //gForceZ = accelZ / constante_rango_accel;
  
  rotX = (gyroX / constante_rango_gyro ) - x_cal;                              //le restamos el angulo del giroscopio en el calibrado
  //rotZ = (gyroZ / constante_rango_gyro ) - z_cal;

  x_ang += rotX*tiempo_ciclo;                                                  //calculamos el angulo integrando la velocidad angular
  //z_ang += rotZ*tiempo_ciclo;

  actotal = sqrt((accelX*accelX) + (accelY*accelY) + (accelZ*accelZ));         //calculamos el vector aceleración total para obtener el angulo segun la gravedad
  
  gx_ang = asin( accelY/actotal )* 61.2 + gx_ang_inicial ;                     //calculamos el angulo de cada eje restandole el valor inicial de cada eje
  //gz_ang = acos( accelZ/actotal )* 61.2 + gz_ang_inicial ;



  x_ang = (x_ang*0.98) + (gx_ang*0.02);                                        //juntamos los angulos del acelerómetro y el gyroscopio para evitar las fluctuaciones y el 'drift'
  //z_ang = (z_ang*0.95) + (gz_ang*0.05); 
}

void calculo_PID()
{
  
  error_actual_x = angulo_x_deseado - x_ang;
  //error_actual_z = angulo_z_deseado - z_ang;
      
  //calculo de la respuesta proporcional
  
  rp_x = error_actual_x * Kp ;
  //rp_z = error_actual_z * Kp ;

  // calculo de la respuesta integral
  
  error_int_x += error_actual_x * tiempo_ciclo;
  //error_int_z += error_actual_z * tiempo_ciclo;
    
  ri_x = error_int_x * Ki;
  //ri_z = error_int_x * Ki;
  
  // calculo de la respuesta diferencial
  
  rd_x = -rotX *Kd;
  //rd_z = -rotZ *Kd;

   //juntamos todas las respuestas en una para despues usarla para corregir la posicion

   r_x = rp_x + ri_x + rd_x;
   //r_z = rp_z + ri_z + rd_z;
  
  //pasamos las variables actuales a las pasadas para volver a empezar el bucle

   error_pasado_x = error_actual_x ;
   //error_pasado_z = error_actual_z ;
  
}


void correccion_posicion()
{
  r_x = constrain ( r_x, -v_max , v_max );
  
  if ( r_x < 0 )
  {
     if(x_ang > 60)
     {
     analogWrite(motor_dch_atras ,0);
     analogWrite(motor_dch_alante ,0);
     analogWrite(motor_izq_alante ,0);
     analogWrite(motor_izq_atras ,0);
     }
     else if (x_ang > 4)
     {
     r_x = r_x *1.5;
     r_x = constrain ( r_x, -v_max , v_max );
     
     analogWrite(motor_dch_alante ,-r_x);
     analogWrite(motor_dch_atras ,0);
     analogWrite(motor_izq_alante ,-r_x);
     analogWrite(motor_izq_atras ,0);
     
     }
     else if (x_ang > 8)
     {
     r_x = r_x *3;
     r_x = constrain ( r_x, -v_max , v_max );
     
     analogWrite(motor_dch_alante ,-r_x);
     analogWrite(motor_dch_atras ,0);
     analogWrite(motor_izq_alante ,-r_x);
     analogWrite(motor_izq_atras ,0);
     
     }
     else
     {
     r_x = map(r_x , -v_max , 0, -v_max, -r_min);
     
     analogWrite(motor_dch_alante ,-r_x);
     analogWrite(motor_dch_atras ,0);
     analogWrite(motor_izq_alante ,-r_x);
     analogWrite(motor_izq_atras ,0);
     }
  }
  
  if ( r_x > 0 )
  {
     if(x_ang < -60)
     {
     analogWrite(motor_dch_atras ,0);
     analogWrite(motor_dch_alante ,0);
     analogWrite(motor_izq_alante ,0);
     analogWrite(motor_izq_atras ,0);
     }
     else if (x_ang < -4)
     {
     r_x = r_x *1.5;
     r_x = constrain ( r_x, -v_max , v_max );
     analogWrite(motor_dch_atras ,r_x);
     analogWrite(motor_dch_alante ,0);
     analogWrite(motor_izq_alante ,0);
     analogWrite(motor_izq_atras ,r_x);
     }
     else if (x_ang < -8)
     {
     r_x = r_x *3;
     r_x = constrain ( r_x, -v_max , v_max );
     analogWrite(motor_dch_atras ,r_x);
     analogWrite(motor_dch_alante ,0);
     analogWrite(motor_izq_alante ,0);
     analogWrite(motor_izq_atras ,r_x);
     }
     else
     {
     r_x = map(r_x , 0 , v_max , r_min, v_max);
     analogWrite(motor_dch_atras ,r_x);
     analogWrite(motor_dch_alante ,0);
     analogWrite(motor_izq_alante ,0);
     analogWrite(motor_izq_atras ,r_x);
     }
  } 
  if ( r_x == 0 )
  {
      analogWrite(motor_dch_alante ,0);
      analogWrite(motor_dch_atras ,0);
      analogWrite(motor_izq_alante ,0);
      analogWrite(motor_izq_atras ,0);
  }

}


void visualizar_datos() 
{
  Serial.print("Angulo (total) = ");  Serial.print(x_ang); Serial.print("//  Angulo (g)"); Serial.print(gx_ang);  Serial.print("// salida ="); Serial.println(r_x);
}
