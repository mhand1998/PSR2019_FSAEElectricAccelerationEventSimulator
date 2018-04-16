function [idx] = Closest(test_array,test_value)

tmp = abs(test_array - test_value);

[~, idx] = min(tmp);

%close_value = test_array(idx);
end
