function plot_caminhos(P,x,y,Nv)           
    plotarmapa(x,y)
    hold on    
    cores = ['r', 'b' , 'g','m','c','y','k']; % Cores dos plots dos vants   
    for i=1:Nv                   
        x_plot = x(P{i});
        y_plot = y(P{i});       
        plot(x_plot, y_plot,'--s','Color',cores(i),...
            'LineWidth',1,...
            'MarkerSize',10,...
            'MarkerEdgeColor','k',...
            'MarkerFaceColor',cores(i),...
            'DisplayName',"Rota VANT - "+num2str(i));        
        legend('Location', 'southoutside','Orientation','horizontal');             
    end    
    hold off
end
