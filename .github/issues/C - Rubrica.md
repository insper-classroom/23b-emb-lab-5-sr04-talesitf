> Issue:
>
> - Você só deve fechar as issues que foram 100% concluidas.
> - Se não concluiu 100% a issues, você pode fazer um comentário que ireimos analisar!

A entrega do Lab deve ser um sistema que possui um HC-SR04 e faz periodicamente a leitura da distância e exibe a informacão no OLED, vocês devem estruturar a solução utilizando os recursos do RTOS.

## Dicas

Trig:

1. Gerar o pulso no pino de **Trig** com `delay_us`.

Echo:

1. Iniciar o RTT na borda de subida do pino Echo
    - Qual prescale usar?
1. Ler valor do RTT em borda de descida do pino Echo
   - Consulte [a documentaćão do ASF-RTT](https://asf.microchip.com/docs/latest/same70/html/rtt_8c.html) para saber como ler o valor atual do contador.

Dica:
 
 - Nessa primeira etapa não precisamos de nenhuma interrupção do RTT, ele vai funcionar apenas como um relógio. Passe 0 no último parâmetro da função `RTT_Init()`.
    
- Para consultarmos o valor atual do RTT, utilize a funcão `rtt_read_timer_value`
    
```
uint32_t rtt_read_timer_value 	( 	Rtt *  	p_rtt	) 	

Read the current value of the RTT timer value.

Parameters
    p_rtt	Pointer to an RTT instance.

Returns
    The current Real-time Timer value. 

Referenced by configure_rtt(), gpbr_test_configure_rtt(), main(), and refresh_display().
```

Conta: 

1. Realizar o cálculo da distância usando o valor do RTT 
1. Lembre de usar como base de tempo o valor que configurou no RTT.
1. Exiba nos no OLED a distância atual em cm.

OLED:

- Exiba a distância no OLED
