> Issue:
>
> - Você só deve fechar as issues que foram 100% concluidas.
> - Se não concluiu 100% a issues, você pode fazer um comentário que ireimos analisar!

| Projeto |
| ------| 
| `RTOS-AFEC-IRQ`|       

No código do AFEC, vamos adicionar mais um potênciometro e para isso vamos precisar de mais uma entrada analógica. 

Tarefa:

1. Configure um novo pino para funcionar com AFEC (No README do exemplo tem uma explicação)
1. Crie um novo [`Timer`](https://insper.github.io/ComputacaoEmbarcada/navigation/Dicas/Util-freertos/#software-timer)
1. Envio de dados: 
    - Você pode criar uma nova fila para enviar os dados do novo AFEC
    - Você pode modificar o tipo da fila e enviar os dados usando uma única fila
1. A `task_process` recebe o valor dos dois afecs e faz uma média com eles:
    - $valor = (AFEC1 + AFEC0)/2$

Qualidade de Código:

- Lembre de verificar se os testes de qualidade de código estão passando.


