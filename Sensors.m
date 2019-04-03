%Ultrasonic with wall
x1=[9 13 17 21 25 29 33 37 41 45 49 53 57 61 65 69 73 77 81 85 89 93 97 101];
y1=[9.1 13.5 17.2 20.9 24.9 28.8 32.9 37.2 40.9 45.4 49.7 53.4 57.2 61.2 65.6 69.4 73.4 77.2 81 85.3 89.1 93.1 96.9 100.5];

figure
subplot(4,2,1)
scatter(x1,y1)
title('\textbf{Ultrasonic sensor | Distance measurement with wall as object}')
ylabel('Value [cm]')
hold on
plot(x1,x1)
xlim([0 140])
hold off
legend('Measured','Reference')
subplot(4,2,2)
scatter(x1,x1-y1)
title('\textbf{Comparison between fixed and measured value}')
ylabel('Error [cm]')
xlim([0 140])
ylim([-5 5])

%Ultrasonic with block
x2=[4 6 8 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80 85 90 95 100];
y2=[4.2 5.8 8.2 10.3 16.6 21.9 27.7 33.4 39.1 42.0 48.3 53.6 57.8 62.6 67.2 73.0 77.3 82.6 87.9 92.5 97.5 102.4];

subplot(4,2,3)
scatter(x2,y2)
title('\textbf{Ultrasonic sensor | Distance measurement with block as object}')
ylabel('Value [cm]')
hold on
plot(x2,x2)
xlim([0 140])
hold off
legend('Measured','Reference')
subplot(4,2,4)
scatter(x2,x2-y2)
ylabel('Error [cm]')
xlim([0 140])
ylim([-5 5])

%Infrared with wall
x3=[10 12 14 16 18 20 22 24 26 28 30];
y3=[12 15 18 21 23 25 28 30 32 34 37];

subplot(4,2,5)
scatter(x3,y3)
title('\textbf{Infrared sensor | Distance measurement with wall as object}')
ylabel('Value [cm]')
hold on
plot(x3,x3)
xlim([0 40])
hold off
legend('Measured','Reference')
subplot(4,2,6)
scatter(x3,x3-y3)
ylabel('Error [cm]')
xlim([0 40])
ylim([-10 10])

%Infrared with block
x4=[5 7 9 11 13 15 17 19 21 23 25 27];
y4=[0 8 14 19 23 29 35 40 45 50 55 inf];

subplot(4,2,7)
scatter(x4,y4)
title('\textbf{Infrared sensor | Distance measurement with block as object}')
xlabel('Effective distance [cm]')
ylabel('Value [cm]')
hold on
plot(x4,x4)
xlim([0 40])
hold off
legend('Measured','Reference')
subplot(4,2,8)
scatter(x4,x4-y4)
xlabel('Effective distance [cm]')
ylabel('Error [cm]')
xlim([0 40])
ylim([-40 10])
