Conecte o sensor na placa no microcontrolador:

Setup:

- Fazer a montagem na protoboard
- Escolher dois pinos para Echo e Trig
- Configurar Trig como output e Echo como input
- Configurar irq de boarda no pino Echo
    - Lembre de criar a função de callback.

**Warning: Não ative PULL_UP no pino do ECHO!**
