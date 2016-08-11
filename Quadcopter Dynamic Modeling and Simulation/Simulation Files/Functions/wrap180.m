%% Wrap 180 angle degree function
% for Euler Angles

function [output] = wrap180(input)

for i = 1:length(input)
   
    while(input(i) >= 180)
        input(i) = input(i) - 360;
    end
            
    while(input(i) < -180)
        input(i) = input(i) + 360;
    end
      
end
    output = input;
end