function CarData = CarTracker(CarData, CurrTime, DetectionsList, DetectionsTime, nDetections, ActionList, serialPort)
nCars = 1;
minVel4az = 10;

if isempty(CarData)
    
    CarData(1).state = [0 0 0 0 DetectionsTime];
    CarData(1).az = 0;
    CarData = repmat(CarData,1,nCars);
end

for ind = 1:nCars
    dT = DetectionsTime - CarData(ind).state(5);
    CarData(ind).predNextState = CarData(ind).state + [CarData(ind).state(3:4)*dT 0 0 dT];
end
distMat = inf(nDetections, nCars);
for ii = 1:nDetections
    for carInd = 1:nCars
        distMat(ii,carInd) = norm(CarData(carInd).predNextState(1:2) - DetectionsList(ii,1:2));
    end
end

while nDetections>nCars
    [tmp,ind2del] = max(min(distMat,[],2));
    distMat(ind2del,:) = [];
    DetectionsList(ind2del,:) = []; 
    nDetections = nDetections-1;
end

permss = perms(1:nCars);
permss = permss(:,1:nDetections);
permssDist = zeros(size(permss,1),1);

for ii = 1:size(permss,1)
    for detInd=1:nDetections
        permssDist(ii) = permssDist(ii)+ distMat(detInd, permss(ii,detInd));
    end 
end
[tmp, minPerm] = min(permssDist);

for ii = 1:nDetections
    dT = DetectionsTime - CarData(permss(minPerm, ii)).state(5);
    if dT>0
        CarData(permss(minPerm, ii)).state = [DetectionsList(ii,1:2) (DetectionsList(ii,1:2)-CarData(permss(minPerm, ii)).state(1:2))/dT DetectionsTime];
    else
        CarData(permss(minPerm, ii)).state = [DetectionsList(ii,1:2) 0 0 DetectionsTime];
    end
    if norm( CarData(permss(minPerm, ii)).state(3:4)) > minVel4az
        CarData(permss(minPerm, ii)).az = atan2(CarData(permss(minPerm, ii)).state(4), CarData(permss(minPerm, ii)).state(3))*180/pi;
        if CarData(permss(minPerm, ii)).az < 0 
            CarData(permss(minPerm, ii)).az = CarData(permss(minPerm, ii)).az+180;
        end
    end
end

dt =  DetectionsTime - CurrTime;
for ind = 1:nCars
    CarData(ind).predState = CarData(ind).state + [CarData(ind).state(3:4)*dt 0 0 dt];
end
for carInd = 1:nCars
    if ActionList(carInd,3)>0
        CarData(carInd).reqPos = ActionList(carInd,1:2);
    end
    if ~isempty(serialPort)
        CarData = sendCommand(carInd,CarData,serialPort);
    end
end
end

