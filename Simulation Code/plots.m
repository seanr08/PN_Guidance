% Plots

% Effective Lethal Radius for Fragmentation Warhead (in ft)
R_e = 1.8*3.28; % Adjust based on warhead specs or test data

comp_miss_vec = miss_vec(1:length(miss_vec)-33);
new_miss_vec = comp_miss_vec(1:length(comp_miss_vec)-57);

% Compute Pk using fragmentation lethality model
Pk_vec_new = 1 - exp(-(R_e ./ new_miss_vec).^2);

Success_rate = mean(1 - exp(-(R_e ./ comp_miss_vec).^2));

figure;
plot(new_miss_vec,Pk_vec_new.*100)
title('Probability of Kill vs Miss Distance')
xlabel('Miss Distance (ft)')
ylabel('Probability of Kill')

% Plot Normal Distribution of Miss
miss_x=[min(new_miss_vec):0.01:max(new_miss_vec)];
miss_y=normpdf(miss_x,mean(new_miss_vec),std(new_miss_vec));
figure;
plot(miss_x,miss_y);
title('Distribution of Miss Distance')
xlabel('Miss Distance')
ylabel('Probability Density')

fprintf('Mean Miss Distance: %.4f\n', mean(new_miss_vec));
fprintf('Median Miss Distance: %.4f\n', median(new_miss_vec));
fprintf('Standard Deviation of Miss: %.4f\n', std(new_miss_vec));
fprintf('Mean Probability of Kill (Fragmentation): %.4f\n', mean(Pk_vec_new));
fprintf('Median Probability of Kill (Fragmentation): %.4f\n', median(Pk_vec_new));
fprintf('Standard Deviation of Probability of Kill: %.4f\n', std(Pk_vec_new));
fprintf('Number of Complete Misses: %.4f\n', complete_miss);
fprintf('Predicted Success Rate: %.4f\n', Success_rate);