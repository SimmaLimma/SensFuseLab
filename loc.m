function  estimation = loc(rphat, sm, type)
%LOC Location estimation of object
%   Location estimation with different methods
%   Outputs location estimation for all time instances

if(type == 'tdoa2')
    
    %Store start pos
    startPos = sm.x0;
    
    
    %
    rphatPair = zeros(88,28);
    
    rowInd = 1;
    for i = 1:7
        for j = i+1:8
            rphatPair(:,rowInd) = rphat(:,i) - rphat(:,j);
            rowInd = rowInd +1;
        end
    end
    
    
    % Var for location estimation at each time instance
    estimation = zeros(2,88);
    
    
    % Using TDOA2 Pairwise approach (i.e. tdoa2)
    for tp = 1:length(rphatPair)
        tp
        
        %Momentanues Sig object
        tmpSig = sig(rphatPair(tp,:));
        
        %Estmation and update estimated sensmordel
        shat = estimate(sm, tmpSig);
        sm.x0 = shat.x0;
        
        %Updating output vector
        estimation(:,tp) = shat.x0;
    end
    
    
    %Reset start position
    sm.x0 = startPos;
end



% TODO: Remember to COPY sensmod object, since I think it is by reference



end

