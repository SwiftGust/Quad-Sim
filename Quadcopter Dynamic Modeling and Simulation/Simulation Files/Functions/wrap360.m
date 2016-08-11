%% Wrap 180 angle degree function
% for Euler Angles

function [output] = wrap360(input)

for i = 1:length(input)
   
    while(input(i) >= 360)
        input(i) = input(i) - 360;
    end
            
    while(input(i) < -360)
        input(i) = input(i) + 360;
    end
      
end
    output = input;
end