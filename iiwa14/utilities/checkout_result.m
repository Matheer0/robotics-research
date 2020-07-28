function [ave, minimum, maximum] = checkout_result(manipulability_measures)
    
    ave = mean(manipulability_measures);
    minimum = min(manipulability_measures);
    maximum = max(manipulability_measures);

end