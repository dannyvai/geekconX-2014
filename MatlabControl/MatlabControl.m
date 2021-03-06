% clear
nCars=1;
% serialPort=[];
if 0
    echoudp('on',5001)
    u = udp('10.0.0.1',5001,'LocalHost','10.0.0.2','LocalPort',5001,'Timeout',0.1);
    fopen(u)
    serialPort = serial('Com7','Timeout',0.01);
    fopen(serialPort);

end
DetectionsTime = 10;
DetectionsList = [1 1];
nDetections = 1;
ActionList=zeros(nCars,3);
% ActionList(1,:) = [100 100 1];
ActionList(1,:) = [300 100 1];
CarData=[];
figure;
hold on;
for id = 1:nCars
    h_p(id) = plot(0,0,'o','markersize',10,'markerfacecolor',[0 0 1]);
    h_t(id) = text(0,0,num2str(id));
end
axis([0 640 0 480]);
dt = 0.1;
sTime =tic; 
%for CurrTime = DetectionsTime+0.05: dt: DetectionsTime+160
 while 1  
     CurrTime =toc(sTime);
      fclose(u)
     pause(0.05);
      fopen(u)

    A = fread(u,200,'int32');
    disp(A)
    if numel(A)>0 && mod(numel(A),2)==0
        
        DetectionsList = reshape(A,2,numel(A)/2)';
        nDetections = numel(A)/2;
    else
        DetectionsList=[];
        nDetections = 0;
    end
    
CarData = CarTracker(CarData, CurrTime, DetectionsList, CurrTime, nDetections, ActionList, serialPort);
for id = 1:nCars
    set(h_p(id),'xdata',CarData(id).state(1),'ydata',CarData(id).state(2));
    set(h_t(id),'position',CarData(id).state(1:2)+10);
end

nDetections = 0;

% while (toc(sTime)<dt) 
% pause(0.01);
% end;

end
if 0
    
    fclose(serialPort);
    delete(serialPort);
    echoudp('off')
    fclose(u)
    delete(u);
end