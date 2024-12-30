function plotando_caminhos_aco_puro(P,Nv,x,y,numfig)
  hold off
  figure(numfig); % definindo o número da figura
  % plotMap(x,y,numfig) % Plotando o Mapa
  hold on
  cores = ['b' , 'g','m','c','y','k']; % Cores dos plots dos vants
  for i=1:Nv
      x_plot = x(P{i});
      y_plot = y(P{i});
      plot(x_plot, y_plot,cores(i))
      plot(x_plot(2:end-1),y_plot(2:end-1),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0,0,0], 'MarkerEdgeColor','k','MarkerFaceColor',cores(i));
  end
end