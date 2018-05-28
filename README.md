# KUKA-KR3-R540-Simulação-Matlab
O presente projeto apresenta uma simulação do robô Kuka KR3 R540, desenvolvida no software Matlab através da toolbox de robótica criada por Peter Corke. É um trabalho promovido pela disciplina de Introdução À Robótica Industrial, no curso de Engenharia de Controle e Automação da Universiade Federal de Santa Cantarina - Câmpus Blumenau.

Autores: Ana Júlia Lanzarin e Eduard Hermes Anschau.

Primeiramente atribuímos os parâmetros de DH aos 6 elos do manipulador com a função Link (), fornecida pelo toolbox de robótica de Peter Corke. Para usar esta função é necessário antes resolver a, temos que obter os parâmetros Denavit Hartenberg do manipulador. Com isso, para cada elo, atribuímos os parâmetros nomeados de theta, d, a e alpha. Para exemplificar, para o primeiro elo tem-se:

L(1) = Link('d', -345, 'a', 20, 'alpha', pi/2, 'qlim', [/180 170*pi/180]); 

Para o primeiro elo, como podemos ver, o ângulo da junta foi sumprimido (isso quer dizer que não há offset). A distância de deslocamento 'd' é 345. O comprimento 'a' do elo  é igual a 20. O parâmetro 'alpha' é de 90 graus e theta é limitado por 'qlim' de -170 a 170 graus.

L(1), L(2), L(3), L(4) e L(5) representam os cinco primeiros links e L(6) representa o lik da ferramenta. Para este manipulador todas as juntas são rotacionais.

Posteriormente, utilizamos a função SerialLink (), que cria um robô de links em série com os links criados anteriormente. Para usar esta função, devemos criar um vetor coluna com os links. No nosso caso, este vetor é dado por:

L = [L(1);L(2);L(3);L(4);L(5);L(6)] 

E chamamos a função deste modo:

KUKA = SerialLink(L, 'name', 'KUKA KR3 R540');

Para plotar a simulação do robô utilizamos a função teach(), assim:

KUKA.teach(Q, 'notiles', 'floorlevel', 156, 'lightpos',[0 0 -20]);

Alteramos alguns parâmetros desta função para melhor visualização gráfica. E além disso, revertemos os eixos Z e X na simulação, de tal modo que o robô fique 'em pé' no gráfico. Essas reversões foram feitas com a seguinte linha de código:

set(gca,'Zdir','reverse','Xdir','reverse');

O usuário do código pode 'brincar' com o robô plotado pela função teach(). 

Além disso, o código apresenta os cálculos da cinemática direta e inversa do robô.

Cria-se as matrizes homogêneas com as funções do toolbox para o cálculo da cinemática direta, do seguinte modo:

T1 = L(1).A(q1).T;

T1 se refere, no código, à matriz de transformação homogênea do referencial 0 para o referencial 1. E assim criamos as matriz T2, T3, T4, T5 e T6.

O usuário do código pode alterar os valores de q1, q2, q3, q4, q5 e q6 para a cinemática direta, para ver se os valores conferem com os resultados obtidos pelo cálculo da cinemática inversa.





