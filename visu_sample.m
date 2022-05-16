clear all

pic_size_val1=600;
pic_size_val2=500;

output = 1;
percent = 90;
visu = [0; 0; 0];

if output == 1  
  visu = [100; 0; 0];
  visu = imagesc(visu);
  if percent < 80
  visu.AlphaData = 0.4;
  end
  xlabels = 'O';  %time labels
  ylabel('Classified area');
  xlabel('Classified area');    
 set(gca, 'XTick', -1, 'XTickLabel', xlabels);
 set(gca, 'YTick', 1, 'YTickLabel', '1. Area');

elseif output == 2
  visu = [0; 100; 0];
  visu = imagesc(visu);
  truesize([pic_size_val1 pic_size_val2]);
  if percent < 80
  visu.AlphaData = 0.4;
  end
  xlabels = 'O';  %time labels
  ylabel('Classified area');
  xlabel('Classified area');    
 set(gca, 'XTick', -1, 'XTickLabel', xlabels);
 set(gca, 'YTick', 2, 'YTickLabel', '2. Area');
elseif output == 3 
  visu = [0; 0; 100];
  visu = imagesc(visu);
  truesize([pic_size_val1 pic_size_val2]);
  if percent < 80
  visu.AlphaData = 0.4;
  end
  xlabels = 'O';  %time labels
  ylabel('Classified area');
  xlabel('Classified area');    
 set(gca, 'XTick', -1, 'XTickLabel', xlabels);
 set(gca, 'YTick', 3, 'YTickLabel', '3. Area');
else
  visu = [0; 0; 0];
  visu = imagesc(visu);
  truesize([pic_size_val1 pic_size_val2]);
  xlabels = 'O';  %time labels
  ylabel('Classified area');
  xlabel('Classified area');    
 set(gca, 'XTick', -1, 'XTickLabel', xlabels);
 set(gca, 'YTick', 3, 'YTickLabel', '1. Area');
end


%%
A=randn(100,200);  
 visu = [0; 0; 100];
 imagesc(visu)
  xlabels = 'O';  %time labels
  ylabel('Classified area');
  xlabel('Classified area');    
 set(gca, 'XTick', -1, 'XTickLabel', xlabels);
 set(gca, 'YTick', 3, 'YTickLabel', '1. Area');
%% Bar graph
X = categorical({'1. Area','2. Area','3. Area'});
X = reordercats(X,{'1. Area','2. Area','3. Area'});
Y = [0 0 0];

if output == 1
    visu = [100; 0; 0];
elseif output == 2
    visu = [0, 100, 0];
elseif output == 3 
    visu = [0; 0; 100];
else
    visu = [0; 0; 0];
end

Y = [10 21 33];
bar(X,Y)

%% Sample

data = [100; 0; 0];
figure; hAxes = gca;
imagesc( hAxes, data );
colormap( hAxes , [1 1 1; 1 0 0; 0 1 0] )
