#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <stdlib.h>
#include <math.h>

// === Endereço e registradores do MMA8451Q ===
#define MMA8451Q_I2C_ADDR    0x1D
#define MMA8451Q_CTRL_REG1   0x2A

// === Bits de configuração ===
#define MMA8451Q_ACTIVE_BIT  0x01
#define MMA8451Q_ODR   (0x0 << 3)  // 100 Hz conforme datasheet (DR=100b)

// === Obter dispositivos ===
static const struct device *const accel = DEVICE_DT_GET(DT_ALIAS(accel0));
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static int64_t delta_t = 0 ;
static struct sensor_value x, y, z;
static double Ax, Ay, Az;
static float Vx=0, Vy=0, Vz=0, Px=0, Py=0, Pz=0, Vxo=0, Vyo=0, Vzo=0, Axo=0, Ayo=0, Azo=0;

void mma8451q_configurar_odr(void)
{
    uint8_t buf[2];
    int ret;

    // 1️⃣ Colocar o sensor em standby (necessário antes de mudar ODR)
    buf[0] = MMA8451Q_CTRL_REG1;
    buf[1] = 0x00;
    ret = i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);
    if (ret) {
        printk("ERRO ao colocar MMA8451Q em standby (%d)\n", ret);
        return;
    }

    // 2️⃣ Configurar ODR = 100 Hz (bits DR[5:3] = 100)
    buf[0] = MMA8451Q_CTRL_REG1;
    buf[1] = MMA8451Q_ODR;
    ret = i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);
    if (ret) {
        printk("ERRO ao configurar ODR (%d)\n", ret);
        return;
    }

    // 3️⃣ Ativar o sensor novamente
    buf[0] = MMA8451Q_CTRL_REG1;
    buf[1] = MMA8451Q_ODR | MMA8451Q_ACTIVE_BIT;
    ret = i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);
    if (ret) {
        printk("ERRO ao ativar MMA8451Q (%d)\n", ret);
        return;
    }

    printk("MMA8451Q configurado para 100 Hz via I2C.\n");
}

void terminal(void) {
    if (!device_is_ready(accel)) {
        printk("Sensor MMA8451Q não está pronto!\n");
        return;
    }

    if (!device_is_ready(i2c_dev)) {
        printk("I2C não está pronto!\n");
        return;
    }

    printk("Configurando ODR via I2C...\n");
    mma8451q_configurar_odr();

    // Pequeno atraso para estabilizar após reativar
    k_msleep(200);

    printk("Iniciando leituras contínuas...\n");

    while (1) {

        printk("AX=%f | AY=%f | AZ=%f\n",
                Ax, Ay, Az);
        printk("VX=%f | VY=%f | VZ=%f\n",
                Vx, Vy, Vz);
        printk("X=%f | Y=%f | Z=%f\n\n",
                Px, Py, Pz);

        // Intervalo aproximado para 100 Hz (10 ms por amostra)
        k_msleep(100);
    }
}

float trapezio(double If, double I0){
    if(If+I0 <1 && If+I0 > -1) return 0;
    return delta_t*(If+I0)/2;
}

void aquisitor(void)
{
    while(1){
        int ret = sensor_sample_fetch(accel);
        if (ret) {
            printk("Erro sensor_sample_fetch (%d)\n", ret);
            k_msleep(20);
            continue;
        }

        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &x);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &y);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &z);

        Ax = sensor_value_to_double(&x);
        Ay = sensor_value_to_double(&y);
        Az = sensor_value_to_double(&z);

        k_msleep(5);
    }
}

void transformador(void){

    int64_t t_ms, t_ant = 0;
    while(1){

        t_ms = k_uptime_get();
        delta_t = t_ms - t_ant;
        t_ant = t_ms;

        Vx += trapezio(Ax,Axo);
        Vy += trapezio(Ay,Ayo);
        Vz += trapezio(Az,Azo);

        Px += trapezio(Vx,Vxo);
        Py += trapezio(Vy,Vyo);
        Pz += trapezio(Vz,Vzo);

    
        Axo = Ax;
        Ayo = Ay;
        Azo = Az;

        Vxo = Vx;
        Vyo = Vy;
        Vzo = Vz;
        k_msleep(5);
    }
}

K_THREAD_DEFINE(aquisitor_id, 1024, aquisitor, NULL, NULL, NULL, 2, 0, 0);
K_THREAD_DEFINE(terminal_id, 1024, terminal, NULL, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(transformador_id, 1024, transformador, NULL, NULL, NULL, 3, 0, 0);
