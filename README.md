
## Manipulador Robótico Comau SmartSix - Controle via Matlab e CoppeliaSim

Este projeto foi desenvolvido como parte da disciplina **Manipuladores Robóticos (ELE041/EEE935)** da Universidade Federal de Minas Gerais (UFMG). O objetivo principal é modelar e controlar o robô Comau SmartSix utilizando o **Matlab** e o **Robotics Toolbox**, além de realizar a simulação no ambiente **CoppeliaSim**.

### Funcionalidades:

1. **Modelagem Cinemática do Robô**:
   - O robô foi modelado utilizando a convenção de Denavit-Hartenberg (DH) com base nos arquivos fornecidos em PDF.
   - A configuração inicial é definida pelas juntas: `q = [0; 0; -90º; 0; -90º; 0]`.

2. **Controle de Regulação**:
   - O robô é comandado a atingir uma sequência de posições constantes no espaço, mantendo uma orientação fixa, utilizando a **jacobiana geométrica** e o **erro de orientação eixo-ângulo**.

3. **Controle de Seguimento de Trajetória**:
   - O robô realiza dois círculos no plano YZ a partir da posição final da sequência de regulação, mantendo a orientação constante.

4. **Simulação no CoppeliaSim**:
   - A sequência de juntas é gravada numa matriz `q_seq` e utilizada para comandar o robô virtual no ambiente CoppeliaSim, reproduzindo os movimentos realizados no Matlab.

### Pré-requisitos:

- Matlab com Robotics Toolbox instalado.
- CoppeliaSim Robotics versão 4.3.0.
- Arquivos fornecidos no Moodle, como `rotm2axang2.m`, `control.m`, e `smartsix.ttt`.

### Como executar:

1. **Modelagem e controle no Matlab**: Execute os scripts de controle e verifique a movimentação do robô.
2. **Simulação no CoppeliaSim**:
   - Carregue o arquivo `smartsix.ttt` no CoppeliaSim e inicie a simulação.
   - No Matlab, execute o script `reproduçãoJuntasCoppelia` para comandar o robô no CoppeliaSim com base em `q_seq`.

### Resultados esperados:

- O robô deverá alcançar as posições especificadas com precisão e realizar os movimentos circulares conforme a trajetória definida, tanto no Matlab quanto no CoppeliaSim.

