figure
subplot(2,2,1)
plot(th,xh(1,:),'linewidth',2);
xlabel('T (s)');
ylabel('v_{x} (m/s)');
subplot(2,2,2)
plot(th,xh(6,:),'linewidth',2);
xlabel('T (s)');
ylabel('Y (m)');
subplot(2,2,3)
plot(th,uh(1,:),'linewidth',2);
xlabel('T (s)');
ylabel('a_{x} (m/s)');
subplot(2,2,4)
plot(th,uh(2,:),'linewidth',2);
xlabel('T (s)');
ylabel('\delta_{f} (m)');

figure;
plot(th,xh(2,:),'linewidth',2);
xlabel('T (s)');
ylabel('v_{y} (m/s)');

figure;
plot(1:length(sample_time),sample_time,'linewidth',2);
xlabel('Sampling Instance');
ylabel('Optimization Time(s)');