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
> - Não usar o motor em openloop!! Se precisar, use por menos de 1 minuto
> - É importante seguir a ordem da rotina para garantir que tudo vá funcionar corretamente. Se os parametros de alinhamento de FOC forem incluídos antes de realizar algum alinhamento do motor, não vai funcionar bem.
> - Para listar as tags, basta digitar `git tag` no terminal

## Rotina para usar os motores:

### 1. Verificar conexões dos fios

driver + 2 motores + encoder + bateria

### 2. Testar se motor está funcionando

Acesse o código das respectivas tags do git para testar cada motor:

- teste-motor-0
- teste-motor-1

Ao abrir o Serial Monitor, as primeiras informações devem ser algo parecido com:

```txt
MOT: Enable driver.
MOT: Align sensor.
MOT: sensor_direction==CW
MOT: PP check: OK!
MOT: Zero elec. angle: 4.08
MOT: Align current sense.
CS: Inv B
MOT: Success: 3
MOT: Ready.
```

Importante:
- `sensor_direction` sempre deve ser `CW`, pois o pino `DIR` do encoder AS5600 está conectado no 3.3V
- Depois de `Align current sense.`, deve aparecer somente `CS: Inv B`. Se estiver diferente, conferir se há algum mal contato dos fios do motor no driver ou se o driver não está danificado
- As informações depois do alinhamento do motor são mostradas como:
```<instante (segundos)> <entrada (A)> <iq (A)> <id (A)> <magnitude da corrente  no motor (|A|)> <angulo do motor (rad)> <velocidade do motor (rad/s)>```
- A entrada é corrente (A). Se for diferente de zero, o motor deve se mover. Se for igual a 0, o motor fica parado e se você mexer com a mão, ele não oferece força contrária

### 3. Testar ou ajustar controle de torque

Acesse o código das respectivas tags do git para testar cada motor:

- controle-torque-motor-0
- controle-torque-motor-1

Ao abrir o Serial Monitor, as primeiras informações devem ser algo parecido com:

```txt
MOT: Enable driver.
MOT: Align sensor.
MOT: sensor_direction==CW
MOT: PP check: OK!
MOT: Zero elec. angle: 4.08
MOT: Align current sense.
CS: Inv B
MOT: Success: 3
MOT: Ready.
```

Importante:
- `sensor_direction` sempre deve ser `CW`, pois o pino `DIR` do encoder AS5600 está conectado no 3.3V
- Depois de `Align current sense.`, deve aparecer somente `CS: Inv B`. Se estiver diferente, conferir se há algum mal contato dos fios do motor no driver ou se o driver não está danificado
- As informações depois do alinhamento do motor são mostradas como:
```<instante (segundos)> <entrada (A)> <iq (A)> <id (A)> <magnitude da corrente  no motor (|A|)> <angulo do motor (rad)> <velocidade do motor (rad/s)>```
- A entrada é corrente (A). Se for diferente de zero, o motor deve se mover. Se for igual a 0, o motor fica parado e se você mexer com a mão, ele não oferece força contrária


### 4. Testar ou ajustar controle de velocidade

Acesse o código das respectivas tags do git para testar cada motor:

- controle-vel-motor-0
- controle-vel-motor-1

Ao abrir o Serial Monitor, as primeiras informações devem ser algo parecido com:

```txt
MOT: Enable driver.
MOT: Align sensor.
MOT: sensor_direction==CW
MOT: PP check: OK!
MOT: Zero elec. angle: 4.08
MOT: Align current sense.
CS: Inv B
MOT: Success: 3
MOT: Ready.
```

Importante:
- `sensor_direction` sempre deve ser `CW`, pois o pino `DIR` do encoder AS5600 está conectado no 3.3V
- Depois de `Align current sense.`, deve aparecer somente `CS: Inv B`. Se estiver diferente, conferir se há algum mal contato dos fios do motor no driver ou se o driver não está danificado
- As informações depois do alinhamento do motor são mostradas como:
```<instante (segundos)> <entrada (A)> <iq (A)> <id (A)> <magnitude da corrente  no motor (|A|)> <angulo do motor (rad)> <velocidade do motor (rad/s)>```
- A entrada é velocidade (rad/s). Se for diferente de zero, o motor deve se mover. Se for igual a 0, o motor fica parado e se você mexer com a mão, ele deve fornecer alguma força contrária para manter o motor parado

### 5. Testar recebimento de dados do Firmware Central

[TODO]

<!-- Verificar montagem dos componentes

Tag recebimento_firm_central -->

### 6. Anotar parâmetros de alinhamento FOC

Acesse o código das respectivas tags do git para testar cada motor: 

- alinhamento-FOC-motor-0
- alinhamento-FOC-motor-1

Ao abrir o Serial Monitor, as primeiras informações devem ser algo parecido com:

```txt
MOT: Enable driver.
MOT: Align sensor.
MOT: sensor_direction==CW
MOT: PP check: OK!
MOT: Zero elec. angle: 4.08
MOT: Align current sense.
CS: Inv B
MOT: Success: 3
MOT: Ready.
```

### 7. Execute novamente o código sem alinhar o motor

Acesse, por exemplo, o código das tags: 

- controle-vel-motor-0
- controle-vel-motor-1

Incluir em `main.cpp` dentro do método `void setup()`, antes de `motor.initFOC()`, as seguintes variaveis:

```cpp
motor.zero_electric_angle = <valor que você achou para Zero elec. angle>;
motor.sensor_direction = <valor que você achou para sensor_direction. Se estiver certo: Direction::CW>;
currentSense.skip_align = true;
```

Ou passe o código a seguir pra `main.cpp` e abra o Serial Monitor, passando a velocidade desejada usando o teclado, digitando `T<valor que você deseja para velocidade>` + clica em Enter:

- Antes de `void setup()`:

```cpp
//Command Settings
float input = 0;                                //Enter "T+speed" in the serial monitor to make the two motors rotate in closed loop
Commander command = Commander(Serial);                    //For example, to make both motors rotate at a speed of 10rad/s, input "T10"
void doTarget(char* cmd) { command.scalar(&input, cmd); }
```

- Ao final de `void setup()`:

```cpp
command.add('T', doTarget, "target velocity");
```

- Dentro de `void loop()`:

```cpp
command.run();
```

> Se testar usando o teclado + Serial Monitor, é importante que não esteja imprimindo nenhuma informação no Serial Monitor. Caso contrário, você não vai conseguir enviar o valor desejado para o motor.

Se estiver tudo OK, só seguir com a preparação do robô.

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