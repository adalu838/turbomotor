function lambda_d = contToDisc(lambda_c)

if lambda_c > 1
    lambda_d = 1;
elseif lambda_c == 1
    lambda_d = 0.6;
else
    lambda_d = 0;
end

end


