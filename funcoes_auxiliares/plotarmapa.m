function plotarmapa(x,y)        
    plot(x(1),y(1),'s','Color','k','LineWidth',1,'MarkerSize',20,'MarkerFaceColor','y') 
    hold on
    plot(x(2:end),y(2:end),'s','Color','k','LineWidth',1,'MarkerSize',10)    
    wp = (1:1:length(x))';
    text(x+0.2,y+0.2,"WP:"+ num2str(wp))

    legend('Galpão','Waypoint','Location', 'southoutside','Orientation','horizontal');             
end