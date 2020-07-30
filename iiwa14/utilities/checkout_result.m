function result = checkout_result(manipulability_measures)
    
    ave = mean(manipulability_measures);
    minimum = min(manipulability_measures);
    maximum = max(manipulability_measures);
    
    result = [ave; minimum; maximum];

end