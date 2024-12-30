function plotando_caminhos_dwa(P,Nv,x,y,Obstaculos)
    hold on 
    cores = ['r', 'b' , 'g','k','c','y','m','g']; % Cores dos plots dos vants   
    for i=1:Nv
        for j=1:length(P{i})-1        
            Pi= [x(P{i}(j)),y(P{i}(j))];
            Pf= [x(P{i}(j+1)),y(P{i}(j+1))];  
            % A_Star_plot(Obstaculos,Pi,Pf,cores(i));    
            
            if (j~=1)&&(Nv~=i)
                plot(Pi(1,1),Pi(1,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0,0,0], 'MarkerEdgeColor','k','MarkerFaceColor',cores(i));
            end
            if (j~=1)&&(Nv==i)
                % plot(Pi(1,1),Pi(1,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0,0,0], 'MarkerEdgeColor','k','MarkerFaceColor',"#77AC30");
                plot(Pi(1,1),Pi(1,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0,0,0], 'MarkerEdgeColor','k','MarkerFaceColor',cores(2));
            end
        end
    end
end