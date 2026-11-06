#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define STACK_SIZE 2048
#define PRIORITY 7

// Endereço e registradores do MMA8451Q
#define MMA8451Q_I2C_ADDR   0x1D
#define MMA8451Q_CTRL_REG1  0x2A
#define MMA8451Q_ACTIVE_BIT 0x01
#define MMA8451Q_ODR        (0x0 << 3) // 100 Hz

// Parâmetros do Filtro e ZUPT (Zero-Velocity Update)
#define N_AMOSTRAS_CALIBRACAO 100 // Amostras para calibração inicial
#define DEADBAND_ACC          0.12f // Limiar para eliminar ruído estático (m/s²)
#define JANELA_ESTAVEL        20    // 20 amostras = 200 ms sem movimento
#define LIMIAR_RUIDO_ZUPT     0.08f // Variação máxima entre amostras para considerar repouso

static const struct device *const accel = DEVICE_DT_GET(DT_ALIAS(accel0));
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

// === Variáveis Globais de Tempo ===
volatile int64_t t_ms = 0;
volatile int64_t to_ms = 0;
volatile float dt_s = 0.01f;

// === Variáveis de Medição e Offsets ===
volatile struct sensor_value ax_struct, ay_struct, az_struct;
volatile float ax = 0.0f, ay = 0.0f, az = 0.0f;
volatile float axo = 0.0f, ayo = 0.0f, azo = 0.0f;
volatile float off_x = 0.0f, off_y = 0.0f, off_z = 0.0f;

// === Variáveis Físicas Instantâneas ===
volatile float vx = 0.0f, vy = 0.0f, vz = 0.0f;
volatile float vox = 0.0f, voy = 0.0f, voz = 0.0f;
volatile float px = 0.0f, py = 0.0f, pz = 0.0f;

// === Flags de Sincronização em Esteira ===
volatile bool dado_novo = false;
volatile bool calc_vel_pronto = false;
volatile bool calc_pos_pronto = false;
volatile bool calibrado = false;
volatile bool em_repouso = false;

// Função para formatar float manualmente no printk (com suporte a sinal negativo)
static void formatar_float(float val, char *sinal, int *inteiro, int *fracao)
{
    if (val < 0.0f) {
        *sinal = '-';
        val = -val;
    } else {
        *sinal = ' ';
    }
    *inteiro = (int)val;
    *fracao = (int)((val - *inteiro) * 1000.0f);
}

void mma8451q_configurar_odr(void)
{
    uint8_t buf[2];
    buf[0] = MMA8451Q_CTRL_REG1; buf[1] = 0x00;
    i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);

    buf[0] = MMA8451Q_CTRL_REG1; buf[1] = MMA8451Q_ODR;
    i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);

    buf[0] = MMA8451Q_CTRL_REG1; buf[1] = MMA8451Q_ODR | MMA8451Q_ACTIVE_BIT;
    i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);
}

// -------------------------------------------------------------------
// THREAD 1: Aquisição de Dados + Calibração + ZUPT Dinâmico
// -------------------------------------------------------------------
void thread_acquisicao_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    if (!device_is_ready(accel) || !device_is_ready(i2c_dev)) {
        printk("Erro: Periféricos I2C/Sensor indisponíveis!\n");
        return;
    }

    mma8451q_configurar_odr();
    k_msleep(200);

    // 1. Calibração Inicial (Placa parada no Boot)
    printk("Calibrando sensor... Mantenha a placa imóvel.\n");
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    for (int i = 0; i < N_AMOSTRAS_CALIBRACAO; i++) {
        if (sensor_sample_fetch(accel) == 0) {
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, (struct sensor_value *)&ax_struct);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, (struct sensor_value *)&ay_struct);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, (struct sensor_value *)&az_struct);

            sum_x += sensor_value_to_float((const struct sensor_value *)&ax_struct);
            sum_y += sensor_value_to_float((const struct sensor_value *)&ay_struct);
            sum_z += sensor_value_to_float((const struct sensor_value *)&az_struct);
        }
        k_msleep(10);
    }

    off_x = sum_x / N_AMOSTRAS_CALIBRACAO;
    off_y = sum_y / N_AMOSTRAS_CALIBRACAO;
    off_z = sum_z / N_AMOSTRAS_CALIBRACAO;
    calibrado = true;
    printk("Calibração OK. Offsets -> X:%.2f Y:%.2f Z:%.2f\n", off_x, off_y, off_z);

    // 2. Loop Principal de Amostragem (100 Hz)
    float ult_raw_x = 0.0f, ult_raw_y = 0.0f, ult_raw_z = 0.0f;
    int contador_estavel = 0;

    while (1) {
        if (sensor_sample_fetch(accel) == 0) {
            to_ms = t_ms;
            t_ms = k_uptime_get();

            if (to_ms > 0) {
                dt_s = (float)(t_ms - to_ms) / 1000.0f;
            }

            axo = ax; ayo = ay; azo = az;

            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, (struct sensor_value *)&ax_struct);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, (struct sensor_value *)&ay_struct);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, (struct sensor_value *)&az_struct);

            float raw_x = sensor_value_to_float((const struct sensor_value *)&ax_struct);
            float raw_y = sensor_value_to_float((const struct sensor_value *)&ay_struct);
            float raw_z = sensor_value_to_float((const struct sensor_value *)&az_struct);

            // Detecção de Repouso para Recalibração Dinâmica (ZUPT)
            float diff_x = fabsf(raw_x - ult_raw_x);
            float diff_y = fabsf(raw_y - ult_raw_y);
            float diff_z = fabsf(raw_z - ult_raw_z);

            ult_raw_x = raw_x; ult_raw_y = raw_y; ult_raw_z = raw_z;

            if (diff_x < LIMIAR_RUIDO_ZUPT && diff_y < LIMIAR_RUIDO_ZUPT && diff_z < LIMIAR_RUIDO_ZUPT) {
                contador_estavel++;
            } else {
                contador_estavel = 0;
                em_repouso = false;
            }

            // Se a placa parar em qualquer nova posição/inclinação por >200 ms:
            if (contador_estavel >= JANELA_ESTAVEL) {
                em_repouso = true;
                // Atualiza a nova referência do peso da gravidade
                off_x = raw_x;
                off_y = raw_y;
                off_z = raw_z;
            }

            // Aplica offset e Zona Morta (Deadband)
            float acc_x_limpa = raw_x - off_x;
            float acc_y_limpa = raw_y - off_y;
            float acc_z_limpa = raw_z - off_z;

            ax = (fabsf(acc_x_limpa) > DEADBAND_ACC) ? acc_x_limpa : 0.0f;
            ay = (fabsf(acc_y_limpa) > DEADBAND_ACC) ? acc_y_limpa : 0.0f;
            az = (fabsf(acc_z_limpa) > DEADBAND_ACC) ? acc_z_limpa : 0.0f;

            dado_novo = true; // Notifica a Thread 2
        }

        k_msleep(10);
    }
}

// -------------------------------------------------------------------
// THREAD 2: Cálculo da Velocidade Instantânea 
// -------------------------------------------------------------------
void thread_calculo_velocidade(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    while (1) {
        if (dado_novo && !calc_vel_pronto && calibrado) {
            vox = vx; voy = vy; voz = vz;

            if (em_repouso) {
                // Trava a deriva zerando a velocidade na parada
                vx = 0.0f;
                vy = 0.0f;
                vz = 0.0f;
            } else {
                // Integração do Trapézio: v = v_ant + (a + a_ant) * dt / 2
                vx += (ax + axo) * dt_s / 2.0f;
                vy += (ay + ayo) * dt_s / 2.0f;
                vz += (az + azo) * dt_s / 2.0f;
            }

            calc_vel_pronto = true; // Notifica a Thread 3
        }

        k_msleep(1);
    }
}

// -------------------------------------------------------------------
// THREAD 3: Cálculo da Posição Instantânea (Regra do Trapézio)
// -------------------------------------------------------------------
void thread_calculo_posicao(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    while (1) {
        if (calc_vel_pronto && !calc_pos_pronto && calibrado) {
            
            // Integração do Trapézio: p = p_ant + (v + v_ant) * dt / 2
            px += (vx + vox) * dt_s / 2.0f;
            py += (vy + voy) * dt_s / 2.0f;
            pz += (vz + voz) * dt_s / 2.0f;

            calc_pos_pronto = true; // Notifica a Thread 4 (Serial)
        }

        k_msleep(1);
    }
}

// -------------------------------------------------------------------
// THREAD 4: Envio Serial (CSV: TIMESTAMP, AX, AY, AZ, VX, VY, VZ, PX, PY, PZ)
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// THREAD 4: Exibição
// -------------------------------------------------------------------
void thread_serial_tx_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    char sax, say, saz, svx, svy, svz, spx, spy, spz;
    int axi, axf, ayi, ayf, azi, azf;
    int vxi, vxf, vyi, vyf, vzi, vzf;
    int pxi, pxf, pyi, pyf, pzi, pzf;

    // Aguarda a calibração inicial antes de começar a imprimir
    while (!calibrado) {
        k_msleep(100);
    }

    // Imprime o cabeçalho das colunas uma única vez
    printk("\n========================================================================================================================\n");
    printk("  TEMPO (ms)  |    ACELERACAO (m/s²)    |     VELOCIDADE (m/s)    |        POSICAO (m)      | NO REPOUSO (ZUPT)?\n");
    printk("========================================================================================================================\n");

    while (1) {
        if (calc_vel_pronto && calc_pos_pronto) {

            // Converte floats em partes inteiras/fracionárias com sinal
            formatar_float(ax, &sax, &axi, &axf);
            formatar_float(ay, &say, &ayi, &ayf);
            formatar_float(az, &saz, &azi, &azf);

            formatar_float(vx, &svx, &vxi, &vxf);
            formatar_float(vy, &svy, &vyi, &vyf);
            formatar_float(vz, &svz, &vzi, &vzf);

            formatar_float(px, &spx, &pxi, &pxf);
            formatar_float(py, &spy, &pyi, &pyf);
            formatar_float(pz, &spz, &pzi, &pzf);

            // Exibição alinhada em linha única por amostragem
            printk("[%6lld ms] | X:%c%d.%03d Y:%c%d.%03d Z:%c%d.%03d | X:%c%d.%03d Y:%c%d.%03d Z:%c%d.%03d | X:%c%d.%03d Y:%c%d.%03d Z:%c%d.%03d | %s\n",
                   t_ms,
                   sax, axi, axf, say, ayi, ayf, saz, azi, azf,
                   svx, vxi, vxf, svy, vyi, vyf, svz, vzi, vzf,
                   spx, pxi, pxf, spy, pyi, pyf, spz, pzi, pzf,
                   em_repouso ? " SIM [Trava VX/VY/VZ]" : " NAO [Em movimento]");

            // Reseta as flags de controle
            calc_vel_pronto = false;
            calc_pos_pronto = false;
            dado_novo = false;
        }

        k_msleep(1);
    }
}

// === Criação Estática das 4 Threads no Zephyr RTOS ===
K_THREAD_DEFINE(thread_acq, STACK_SIZE, thread_acquisicao_entry, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(thread_velocidade, STACK_SIZE, thread_calculo_velocidade, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(thread_posicao, STACK_SIZE, thread_calculo_posicao, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(thread_tx, STACK_SIZE, thread_serial_tx_entry, NULL, NULL, NULL, PRIORITY, 0, 0);