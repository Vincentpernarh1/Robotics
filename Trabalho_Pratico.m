clear

% Parametros DH

q = [0 0 (-pi/2) (pi/2) 0 0 pi];

ds = [0 -0.45 0 0 -0.6471 0 -0.095];

as = [0 0.15 0.59 0.13 0 0 0];

alphas = [pi pi/2 pi -pi/2 -pi/2 pi/2 pi];

for i = 1:7

    L(i) = Revolute('offset', q(i), 'd', ds(i), 'a', as(i), 'alpha', alphas(i));

end


% Levando para a posição inicial

q = [0 0 0 -pi/2 0 -pi/2 0]'; % Posicao incial do robo

robot = SerialLink(L, 'name', 'CS6');

figure(1)

robot.plot(q')

 

 

% Controle de Regulação, pontos desejados

pontos = {[0.700 0 0.700], [1 -0.600 0.300], [1 -0.600 1], [1 0.600 1], [1 0.600 0.300]};

% O ultimo ponto desejado é o inicio do circulo

 

Rd_approach = SO3.Ry(pi/2); % Orientação desejada para o approach

Rd_approach = Rd_approach.R(); %realizando a rotação

 

for c = 1:5

    pd = pontos{c}; % Executa em sequencia o controle de regulação de cada ponto desejado

    Td = SE3(Rd_approach, pd);

    K = 3; % Define ganho

    epsilon = 1e-3; % Define critério de parada, tinha usando e-3 mas o erro parece grande

    erro_ant = 1;

    erro = 0;

    figure(2);

    robot.plot(q'); % Plot robô na configuração inicial

    hold on

    Td.plot('rgb'); % Plot pose desejada

    while (norm(erro - erro_ant) > epsilon) % Critério de parada

        i = i+1; % contador

        Junta = robot.jacob0(q); % Jacobiana geométrica

        J_reg = Junta(1:6, 2:end);

        T_reg = robot.fkine(q); % Cinemática direta para pegar a pose do efetuador

        p_reg = transl(T_reg); % Translação do efetuador -> posicao atual

        R_reg = SO3(T_reg);

        R_reg = R_reg.R(); % Extrai rotação do efetuador

        p_err = pd - p_reg; % Erro de translação/posição

        nphi = rotm2axang2(Rd_approach * R_reg');

        nphi_err = nphi(1:3) * nphi(4); % Erro de rotação (n*phi)

        erro_ant = erro;

        erro = [p_err'; nphi_err']; % Vetor de erro

        u_reg = pinv(J_reg) * K * erro; % Lei de controle

        q(2:end) = q(2:end) + 0.1 * u_reg;

        q_seqP(:, i) = q(2:end);

 

   

        robot.plot(q');

 

        control_sig(:,i) = u_reg; % Sinal de controle

 

        juntas_sig(:,i) = q; % Posição Juntas

 

        erro_orientacao(1:3,i) = nphi_err;

        err(1,i) = norm(erro); % Erro de posicao (escalar)

        caminho(1:3,i) = p_reg; % Caminho Percorrido pelo efetuador

    end

    hold off

end

 

 

% Controle Trajetória

Rd = SO3.Ry(pi/2); % Rotacao de referencia

Rd = Rd.R(); % pega matriz de rotacao do efetuador

K = 2; % Define ganho

% Generate circular path

center = [1 0 0.65]; % Center of the circle

radius = 0.35; % Radius of the circle

figure(3);

robot.plot(q'); % Plot robô na configuração inicial

hold on

% Parametros de simulacao escolhidos

t_amostragem = 0.1;

t_simulacao = 10;

wn = pi*2*(1/t_simulacao);

indice = 1;

for n = 0:t_amostragem:2*t_simulacao

    indice = indice + 1;

    trajetoria(1) = center(1);

    trajetoria(2) = center(2) - radius*sin(wn*n);   % mudando este sinal muda o sentido do circulo.

    trajetoria(3) = center(3) - radius*cos(wn*n); % Eixo z

    pd = trajetoria;

    % Derivadas

    Dtrajetoria(1) = 0;

    Dtrajetoria(2) = -radius*wn*cos(wn*n);  % consequente do sinal, e

    Dtrajetoria(3) = radius*wn*sin(wn*n);

    % Controle

    Joint = robot.jacob0(q); % Jacobiana geométrica

    J = Joint(1:6,2:end);

    T = robot.fkine(q); % Cinemática direta para pegar a pose do efetuador

    p = transl(T); % Translação do efetuador -> posicao atual

    R = SO3(T);

    R = R.R(); % Extrai rotação do efetuador

    p_err = pd - p; % Erro de translação

    nphi = rotm2axang2(Rd*R');

    nphi_err = nphi(1:3)*nphi(4); % Erro de rotação (n*phi)

    erro = [p_err'; nphi_err']; % Vetor de erro

    feedforward = [Dtrajetoria 0 0 0]';

    u = pinv(J)*(K*erro + feedforward); % Lei de controle

    q(2:end) = q(2:end) + t_amostragem*u;

    q_seqC(:,indice) = q(2:end);    % armazonando as posições

    view(3)

    robot.plot(q');

    control_sig_circulo(:,indice) = u; % Sinal de controle

    juntas_sig_circulo(:,indice) = q; % Posição Juntas

    erro_orientacao_circulo(1:3,indice) = nphi_err; % Erro rotacao

    err_circulo(1,indice) = norm(erro); % Erro de posicao (escalar)

    camimho_circulo(1:3,indice) = p; % Caminho Percorrido pelo efetuador

end

hold off

 

%Tratamento matriz q_seq (conexão Coppelia)

q_seq = [q_seqP, q_seqC];

q_seq = q_seq*(180/pi); % passar para graus


%% Gráficos

% Controle Regulação

figure(4)

hold on

for b = 1:6

    subplot(3,2,b);

plot(control_sig(b,:))

xlabel('Iterações');

ylabel('Sinal de controle: u(rad/s)');

title("Sinal de controle para "+ b + "a"+ " Junta")

 

end

hold off

 

 

figure(5)

hold on

plot(err);

xlabel('Iterações')

ylabel('Norma do erro: |e|')

box off

title('Norma de erro para rastreamento de posição')

hold off

a = ["Roll","Ptch", "Yaw"];

figure(6)

hold on

for d = 1:3

    subplot(3,1,d);

    plot(erro_orientacao(d,:));

    xlabel('Iterações');

    ylabel('erro');

    title(' Erro de orientação de '+ a(d))

    box off

  

end

hold off

 

figure(7)

view(3)

hold on

plot3(caminho(1,:), caminho(2,:), caminho(3,:));

xlabel('X');

ylabel('Y');

zlabel('Z');

title('trajetório realizado pelo efetuador')

box off

hold off



% Controle Trajetoria

figure(8)

hold on

for l = 1:6

    plot(control_sig_circulo(l,:))

    xlabel('Iterações');

    ylabel('Sinal de controle: u(rad/s)');

    title('Sinais de controle para rastreamento de trajetória- circulo')

end

hold off

figure(9)

hold on

plot(err_circulo);

xlabel('Iterações')

ylabel('Norma do erro: |e|')

box off

hold off

figure(10)

hold on

for s = 1:3

    subplot(3,1,s);

    plot(erro_orientacao_circulo(s,:));

    xlabel('Iterações');

    ylabel('erro');

    box off

    title(' Erro de orientação de '+ a(s))

    hold off

end

hold off

figure(11)

view(3)

hold on

plot3(camimho_circulo(1,:), camimho_circulo(2,:), camimho_circulo(3,:));

xlabel('X');

ylabel('Y');

zlabel('Z');

box off

title('trajetória realizado pelo efetuador')

hold off

 

 