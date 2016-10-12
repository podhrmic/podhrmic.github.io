%% Parse FWMAV data logs and display and save them


%% Clear all
clear all;
close all;
clc;

CLOSE = 0;
PRINT = 1;
OBSTACLE = 1;
SHOW_WP = 1;
METERS_INSTEAD_OF_PIXELS = 1;

%% Select file
[FileName,PathName] = uigetfile({'*.csv';'*.mat'},'Choose data');
filename = [PathName, FileName];
[~, name, ext] = fileparts(FileName); 

if strcmp(ext,'.csv')
    % we opened csv
    [Times,xpx,ypx,phideg,delta_left,delta_right,omega_left,omega_right,...
    rule_number,desired_course,course_error] = fwmav_importfile(filename, 2, inf);

    % Save
    save([PathName, strrep(FileName, '.csv','.mat')]);
else if strcmp(ext,'.mat')
    % we opened mat file
    load(filename);
    end
end




%% Display Position
if (CLOSE) 
    close all
end

figure;
if (METERS_INSTEAD_OF_PIXELS)
    K = 0.0033;
    units = 'm';
else
   K = 1; 
   units = 'px';
end

plot(xpx*K, ypx*K,'b-','LineWidth',3);
set(gca,'Ydir','reverse'); % reverse y-axis of the image so it represents what we see
grid on;
hold on;



if (OBSTACLE) % display obstacle
    ox = [320, 320];
    oy = [200,300];
    plot(ox*K,oy*K,'g','LineWidth',4)
end

if (SHOW_WP) % display waypoints
   wpx = [100,500];
   wpy = [250,250];
   plot(wpx*K,wpy*K,'ro','LineWidth',10)
   viscircles([wpx'*K wpy'*K], [50*K; 50*K],'LineStyle','--','LineWidth',3,'EdgeColor','k');
end

% highlight beginning and the end
plot(xpx(1)*K, ypx(1)*K, 'mo','LineWidth',10)
plot(xpx(end)*K, ypx(end)*K, 'co','LineWidth',10)

if (PRINT)
    set(gca,'FontSize',20)
    xlhand = get(gca,'xlabel');
    set(xlhand,'string',['x[',units,']'],'fontsize',20) 
    ylhand = get(gca,'ylabel');
    set(ylhand,'string',['y[',units,']'],'fontsize',20) 
else
    xlabel(['x[',units,']'])
    ylabel(['y[',units,']'])
end

axis([0 640*K 0 480*K])
title('Position')

if (OBSTACLE)
    legend('Path','Obstacle','Waypoints','Start','End')    
else
    legend('Path','Waypoints','Start','End')
end

axis equal 

if (PRINT)
    saveas(gcf,[filename(1:end-4) '-position'],'png')
end

%% Display azimuth
if (CLOSE) 
    close all
end

figure;
plot(Times, phideg,'b-','LineWidth',3);
grid on;
hold on;
if (PRINT)
    set(gca,'FontSize',20)
    xlhand = get(gca,'xlabel');
    set(xlhand,'string','Time[s]','fontsize',20) 
    ylhand = get(gca,'ylabel');
    set(ylhand,'string','Azimuith[deg]','fontsize',20) 
else
    xlabel('Time[s]')
    ylabel('Azimuth[deg]')
end
title({'Orientation','0 deg is facing right, 90 deg is down, -90 deg is up'})

% Filter azimuth
fphideg = phideg*0;
fphideg(1) = phideg(1);
alpha = 0.05;
for k=2:length(phideg)
    
    fphideg(k) = alpha*phideg(k) + (1-alpha)*fphideg(k-1);
end

% Displaty filtered azimuth
plot(Times, fphideg,'r-','LineWidth',3);
legend('\phi raw', '\phi filtered','Location','Best')

if (PRINT)
    saveas(gcf,[filename(1:end-4) '-orientation'],'png')
end

%% Display angular rate
if (CLOSE) 
    close all
end

dphi = diff(fphideg);
dt = diff(Times);
rate = dphi./dt;

% remove the outliers (where we wrap around 180 -> -180deg)
threshold = 50; % rad/s
for k=2:length(rate)-1
    %if (abs(rate(k) - rate(k-1)) > threshold)
    %    rate(k) = (rate(k+1) + rate(k-1))/2;
    %else
    %    
    %end
    if (abs(rate(k)) > 20)
       rate(k) = rate(k-1); 
    end
end

frate = rate*0;
frate(1) = rate(1);
alpha = 0.01;
for k=2:length(rate)
    
    frate(k) = alpha*rate(k) + (1-alpha)*frate(k-1);
end

figure;
plot(Times(1:length(rate)), rate,'b-','LineWidth',1);
grid on;
hold on;
plot(Times(1:length(frate)), frate,'r-','LineWidth',4);
if (PRINT)
   set(gca,'FontSize',20)
    xlhand = get(gca,'xlabel');
    set(xlhand,'string','Time[s]','fontsize',20) 
    ylhand = get(gca,'ylabel');
    set(ylhand,'string','Angular rate[deg/s]','fontsize',20)  
else
    xlabel('Time[s]')
    ylabel('Angular rate[deg/s]')
end
title('Rate')
axis([0 Times(end) -20 20])
legend('Angular rate', 'Filtered rate')

if (PRINT)
    saveas(gcf,[filename(1:end-4) '-ang_rate'],'png')
end

%% Display control inputs
if (CLOSE)
    close all
end

figure
set(gcf, 'Position', get(0, 'Screensize'));

subplot(3,1,1)
plot(Times, delta_left,'b-','LineWidth',3);
hold on;
grid on;
plot(Times, delta_right,'r-','LineWidth',3);
if (PRINT)
    set(gca,'FontSize',20)
    xlhand = get(gca,'xlabel');
    set(xlhand,'string','Time[s]','fontsize',20) 
    ylhand = get(gca,'ylabel');
    set(ylhand,'string','Delta[rad/s]','fontsize',20) 
else
   xlabel('Time[s]')
   ylabel('Delta[rad/s]')
end
set(gca,'YLim',[-0.5 15]);
legend('\delta_L','\delta_R')
title('Control inputs')

subplot(3,1,2)
plot(Times, omega_left,'b-','LineWidth',3);
hold on;
grid on;
plot(Times, omega_right,'r-','LineWidth',3);
if (PRINT)
    set(gca,'FontSize',20)
    xlhand = get(gca,'xlabel');
    set(xlhand,'string','Time[s]','fontsize',20) 
    ylhand = get(gca,'ylabel');
    set(ylhand,'string','Omega[rad/s]','fontsize',20) 
else
   xlabel('Time[s]')
   ylabel('Omega[rad/s]')
end
set(gca,'YLim',[10 33]);
legend('\omega_L','\omega_R')

subplot(3,1,3)
plot(Times, rule_number,'b-','LineWidth',3);
grid on;
hold on;
set(gca,'YLim',[-0.3 3.3]);
set(gca,'ytick',[0:1:3])
rules = [ 'Idle      ';
           'Turn Left ';
           'Turn Right';
           'Go Forward';
            ];
set(gca,'YTickLabel',rules)        

if (PRINT)
    set(gca,'FontSize',20)
    xlhand = get(gca,'xlabel');
    set(xlhand,'string','Time[s]','fontsize',20) 
else
   xlabel('Time[s]')
end
legend('Rule that fired','Location','East')



if (PRINT)
    saveas(gcf,[filename(1:end-4) '-control_inputs'],'png')
end
