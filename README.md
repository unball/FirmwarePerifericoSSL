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
- PlatformIO + VSCode

> :warning: Cuidado!! :warning:
>
> Não usar o motor em openloop!! Se precisar, use por menos de 1 minuto

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

## Notas

### [outubro/2025]

Espressif não está oferecendo suporte para PlatformIO. Se o driver for MKS ESP32 DUAL FOC, para conseguir usar a versão  mais atual da biblioteca SimpleFOC, a configuração do firmware da ESP32 deve ser buscada por versões mantidas pelos usuários.

Portanto, ao abrir o projeto no VSCode, verificar se as configurações em `platformio.ini` estão parecidas com o código a seguir:

```
[env:esp32dev]

platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.30/platform-espressif32.zip
platform_packages=
  framework-arduinoespressif32 @ https://github.com/espressif/arduino-esp32/releases/download/3.3.0/esp32-3.3.0.zip
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps = askuric/Simple FOC @ 2.3.5

```