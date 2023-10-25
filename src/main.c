/*
 * Example RTOS Atmel Studio
 */

#include "conf_board.h"
#include <asf.h>
#include "mcu6050.h"
#include <math.h>
#include "Fusion/Fusion.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

#define TASK_LED_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY (tskIDLE_PRIORITY)

#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_IDX 8u
#define LED_IDX_MASK (1u << LED_IDX)
#define THRESHOLD 2.0  // Defina um threshold baseado em testes empíricos
#define LED_BLINK_DELAY 200 // Defina um delay para o piscar do LED

SemaphoreHandle_t xSemaphore;
/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void pin_toggle(Pio *pio, uint32_t mask);
void LED_init(int estado);
static void configure_console(void);

/************************************************************************/
/* RTOS Hooks                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  for (;;) {
  }
}

extern void vApplicationIdleHook(void) {}

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
  configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

void pin_toggle(Pio *pio, uint32_t mask) {
  if (pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
  else
    pio_set(pio, mask);
}

void mcu6050_i2c_bus_init(void)
{
    twihs_options_t mcu6050_option;
    pmc_enable_periph_clk(ID_TWIHS2);

    /* Configure the options of TWI driver */
    mcu6050_option.master_clk = sysclk_get_cpu_hz();
    mcu6050_option.speed      = 40000;
    twihs_master_init(TWIHS2, &mcu6050_option);

    /** Enable TWIHS port to control PIO pins */
    pmc_enable_periph_clk(ID_PIOD);
    pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 28);
    pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 27);
}


int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int32_t ierror = 0x00;

    twihs_packet_t p_packet;
    p_packet.chip         = dev_addr;
    p_packet.addr[0]      = reg_addr;
    p_packet.addr_length  = 1;
    p_packet.buffer       = reg_data;
    p_packet.length       = cnt;

    ierror = twihs_master_write(TWIHS2, &p_packet);

    return (int8_t)ierror;
}


int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int32_t ierror = 0x00;

    twihs_packet_t p_packet;
    p_packet.chip         = dev_addr;
    p_packet.addr[0]      = reg_addr;
    p_packet.addr_length  = 1;
    p_packet.buffer       = reg_data;
    p_packet.length       = cnt;

    ierror = twihs_master_read(TWIHS2, &p_packet);

    return (int8_t)ierror;
}

void LED_init(int estado) {
  pmc_enable_periph_clk(LED_PIO_ID);
  pio_set_output(LED_PIO, LED_IDX_MASK, estado, 0, 0);
};

static void configure_console(void) {
  const usart_serial_options_t uart_serial_options = {
      .baudrate = CONF_UART_BAUDRATE,
      .charlength = CONF_UART_CHAR_LENGTH,
      .paritytype = CONF_UART_PARITY,
      .stopbits = CONF_UART_STOP_BITS,
  };

  stdio_serial_init(CONF_UART, &uart_serial_options);
  setbuf(stdout, NULL);
}

/************************************************************************/
/* Tasks                                                                */
/************************************************************************/

static void task_led(void *pvParameters) {
  LED_init(1);
  for (;;) {
    pin_toggle(LED_PIO, LED_IDX_MASK);
    vTaskDelay(1000);
  }
}

static void task_imu ( void * pvParameters ) {
    mcu6050_i2c_bus_init();

    /* buffer para recebimento de dados */
    uint8_t bufferRX[10];
    uint8_t bufferTX[10];

    /* resultado da função */
    uint8_t rtn;

    /* Probing */
    rtn = twihs_probe(TWIHS2, MPU6050_DEFAULT_ADDRESS);
    if(rtn != TWIHS_SUCCESS){
        printf("[ERRO] [i2c] [probe] \n");
    } else {
        printf("[DADO] [i2c] probe OK\n" );
    }

    /* Leitura do registrador WHO_AM_I */
    rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, bufferRX, 1);
    if(rtn != TWIHS_SUCCESS){
        printf("[ERRO] [i2c] [read] \n");
    } else {
        /* Verifica o conteúdo do bufferRX */
        if (bufferRX[0] == 0x68) {  // Substitua 0x68 pelo valor correto do manual
            printf("[SUCESSO] Valor lido é correto: %x\n", bufferRX[0]);
        } else {
            printf("[ERRO] Valor lido é incorreto: %x\n", bufferRX[0]);
        }
    }
    // Set Clock source
    bufferTX[0] = MPU6050_CLOCK_PLL_XGYRO;
    rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, bufferTX, 1);
    if(rtn != TWIHS_SUCCESS)
        printf("[ERRO] [i2c] [write] \n");

    // Aceletromtro em 2G
    bufferTX[0] = MPU6050_ACCEL_FS_2 << MPU6050_ACONFIG_AFS_SEL_BIT; 
    rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, bufferTX, 1);
    if(rtn != TWIHS_SUCCESS)
        printf("[ERRO] [i2c] [write] \n");

    // Configura range giroscopio para operar com 250 °/s
    bufferTX[0] = 0x00; // 250 °/s
    rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, bufferTX, 1);
    if(rtn != TWIHS_SUCCESS)
        printf("[ERRO] [i2c] [write] \n");

    int16_t  raw_acc_x, raw_acc_y, raw_acc_z;
    volatile uint8_t  raw_acc_xHigh, raw_acc_yHigh, raw_acc_zHigh;
    volatile uint8_t  raw_acc_xLow,  raw_acc_yLow,  raw_acc_zLow;
    float proc_acc_x, proc_acc_y, proc_acc_z;

    int16_t  raw_gyr_x, raw_gyr_y, raw_gyr_z;
    volatile uint8_t  raw_gyr_xHigh, raw_gyr_yHigh, raw_gyr_zHigh;
    volatile uint8_t  raw_gyr_xLow,  raw_gyr_yLow,  raw_gyr_zLow;
    float proc_gyr_x, proc_gyr_y, proc_gyr_z;

    float magnitude;
    /* Inicializa Função de fusão */
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs); 
    while(1) {
      // Le valor do acc X High e Low
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &raw_acc_xHigh, 1);
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, &raw_acc_xLow,  1);

      // Le valor do acc y High e  Low
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, &raw_acc_yHigh, 1);
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_yLow,  1);

      // Le valor do acc z HIGH e Low
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, &raw_acc_zHigh, 1);
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_zLow,  1);

      // Dados são do tipo complemento de dois
      raw_acc_x = (raw_acc_xHigh << 8) | (raw_acc_xLow << 0);
      raw_acc_y = (raw_acc_yHigh << 8) | (raw_acc_yLow << 0);
      raw_acc_z = (raw_acc_zHigh << 8) | (raw_acc_zLow << 0);

      // Le valor do gyr X High e Low
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, &raw_gyr_xHigh, 1);
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_L, &raw_gyr_xLow,  1);

      // Le valor do gyr y High e  Low
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_H, &raw_gyr_yHigh, 1);
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_yLow,  1);

      // Le valor do gyr z HIGH e Low
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &raw_gyr_zHigh, 1);
      mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_zLow,  1);

      // Dados são do tipo complemento de dois
      raw_gyr_x = (raw_gyr_xHigh << 8) | (raw_gyr_xLow << 0);
      raw_gyr_y = (raw_gyr_yHigh << 8) | (raw_gyr_yLow << 0);
      raw_gyr_z = (raw_gyr_zHigh << 8) | (raw_gyr_zLow << 0);

      // Dados em escala real
      proc_acc_x = (float)raw_acc_x/16384;
      proc_acc_y = (float)raw_acc_y/16384;
      proc_acc_z = (float)raw_acc_z/16384;

      proc_gyr_x = (float)raw_gyr_x/131;
      proc_gyr_y = (float)raw_gyr_y/131;
      proc_gyr_z = (float)raw_gyr_z/131;
      magnitude = sqrtf(proc_acc_x * proc_acc_x + proc_acc_y * proc_acc_y + proc_acc_z * proc_acc_z);
      if(fabsf(magnitude - 9.81) > THRESHOLD) {
          xSemaphoreGive(xSemaphore);  // Libera o semáforo quando uma batida é detectada
      }
      const FusionVector gyroscope = {proc_gyr_x, proc_gyr_y, proc_gyr_z}; 
      const FusionVector accelerometer = {proc_acc_x, proc_acc_y, proc_acc_z};    
      // Tempo entre amostras
      float dT = 0.1;

      // aplica o algoritmo
      FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dT);

      // dados em pitch roll e yaw
      const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

      printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
      vTaskDelay(1);
    }
}

static void task_house_down(void *pvParameters) {
    while(1) {
        if(xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            pin_toggle(LED_PIO, LED_IDX_MASK);  // Aciona o LED
            vTaskDelay(LED_BLINK_DELAY / portTICK_PERIOD_MS);  // Delay entre desligar e ligar
            pin_toggle(LED_PIO, LED_IDX_MASK);  // Desliga o LED
        }
    }
}



/************************************************************************/
/* main                                                                */
/************************************************************************/

int main(void) {
  sysclk_init();
  board_init();

  /* Initialize the console uart */
  configure_console();

  /* Output demo information. */
  printf("-- Freertos Example --\n\r");
  printf("-- %s\n\r", BOARD_NAME);
  printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
  xSemaphore = xSemaphoreCreateBinary();  // Cria o semáforo
  if (xSemaphore == NULL) {
      printf("Falha na criacao do semaforo\n");
  }

  if (xTaskCreate(task_house_down, "House Down", TASK_LED_STACK_SIZE, NULL,
                  TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
      printf("Failed to create test house down task\r\n");
  }
  /* Create task to make led blink */
  if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
                  TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  }
  if (xTaskCreate(task_imu, "Imu", TASK_LED_STACK_SIZE, NULL,
                  TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test imu task\r\n");
  }
  /* Start the scheduler. */
  vTaskStartScheduler();

  /* Will only get here if there was insufficient memory to create the idle
   * task. */
  return 0;
}
