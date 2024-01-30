function pts2img_4Traning()
% pathInput = 'C:\Users\administration\OneDrive - cqu.edu.cn\TempShare\pcd_rgb_Denoise_SOR';
% pathOutput = 'C:\Users\administration\OneDrive - cqu.edu.cn\TempShare\pcd2img';
pathInput = 'F:\Modeling\extract\35kv_001\pcd_rgb_Denoise_SOR';
pathOutput = 'F:\Modeling\extract\35kv_001\pcd2img';
cd(pathInput)
listPCD = dir('*.pts');
AZs = -30:10:30; % azimuth
ELs = -40:10:40; % elevation

for ii =1:length(listPCD)
%     if ii <= 212
%         continue
%     end
    fprintf('%d in %d \n ', ii,length(listPCD))
    pcdFile = listPCD(ii).name;
    data = importdata( pcdFile );
    dy_dx = getImgSize(data);
%     pcshow(data(:,1:3),data(:,5:7)/255)
    pcshow(data(:,1:3),data(:,4:6)/255)
    fig = gcf;
    fig.InvertHardcopy = 'off';
    set(gcf,'position',[400,200,1000,1000*dy_dx],'color','green' )
    set(gca, 'Color','none')
    axis off
    axis tight
    pathSave = [pathOutput,'/',pcdFile(1:end-4)];
    if ~exist(pathSave, 'dir')
        mkdir( pathSave )
    end
    for i_EL = 1:length(ELs)
        el = ELs(i_EL);
        for i_AZ = 1:length(AZs)
            az = AZs(i_AZ);
            view([az,el]);
            fileSave = [pcdFile(1:end-4),'__az_',num2str(az),'__el_',num2str(el),'.jpg'];
            saveas(gcf,[pathSave,'/',fileSave])
        end
    end
    if mod(ii,20) == 0
        close
    end
end
end

function dy_dx = getImgSize(data)
x_max = max(data(:,1));
x_min = min(data(:,1));
y_max = max(data(:,2));
y_min = min(data(:,2));
dx = x_max - x_min;
dy = y_max - y_min;
dy_dx = ceil( dy/dx) ;
if dy_dx>3
    dy_dx = 3;
end
end