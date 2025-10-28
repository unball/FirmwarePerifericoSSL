# FirmwarePerifericoSSL

Software usado nos drivers do Robô para a categoria SSL (Small Size Soccer)

## Especificações

Hardware:

- Driver: MKS ESP32 DUAL FOC V1.0
- Motor: Gimbal Motor 4108H 120T
- Encoder: AS5600
- Fonte: Bateria de 12V (3 células)

Software: 

- SimpleFOC + Arduino (link para documentação: https://docs.simplefoc.com/)


## Rotina para usar os motores:

### 1. Verificar conexões dos fios

driver + 2 motores + encoder + bateria

### 2. Testar se motor está funcionando

Branch teste-motor-0

Branch teste-motor-1

### 3. Testar ou ajustar controle de torque

Branch controle-torque_motor_0

Branch controle-torque_motor_1

### 4. Testar ou ajustar controle de velocidade

Branch controle-vel_motor_0

Branch controle-vel_motor_1

### 5. Testar recebimento de dados do Firmware Central

Verificar montagem dos componentes

Branch recebimento_firm_central