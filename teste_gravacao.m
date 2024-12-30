clc;
close all;
clear all;

% Criar um objeto VideoWriter
video = VideoWriter('animacao.avi');
video.FrameRate = 10; % Definir taxa de quadros (frames per second)

% Abrir o objeto VideoWriter para escrita
open(video);

% Criar uma figura e um eixo
fig = figure;
axis tight manual;
xlabel('X');
ylabel('Y');

% Loop para gerar frames da animação
for frame = 1:100
    % Código para atualizar a figura para o próximo frame
    plot(rand(1,10));
    title(['Frame ' num2str(frame)]);
    
    % Adicionar o frame ao vídeo
    frame_gravado = getframe(fig);
    writeVideo(video, frame_gravado);
end

% Fechar o objeto VideoWriter
close(video);
