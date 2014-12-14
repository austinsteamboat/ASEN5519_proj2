%% Parse Data
clear all
close all
clc
% 
% fileID = fopen('final_flight_data.txt');
% data_matrix = [];
% for i =1:1
%     C_temp = fgets(fileID);
% end
% 
% i = 1;
% while ~feof(fileID)
%     data_temp = strsplit(fgets(fileID),',');
%     if isempty(strfind(data_temp{1},'ERROR')) && isempty(strfind(data_temp{1},'Starting'))
%         data_matrix = [data_matrix;data_temp];
%         i = i+1;
%     end
% end
% data_new = [];
% for i = 1:size(data_matrix,1)
%     data_vector_temp = [];
%     for j = 2:size(data_matrix,2)
%         data_temp = data_matrix(i,j);
%         data_vector_temp = [data_vector_temp,str2double(data_temp{1})];
%     end
%     data_new = [data_new;data_vector_temp];
% end
% tvec_0 = data_matrix(:,1);
% tvec = [];
% for i = 1:size(tvec_0,1)
%     str_temp = tvec_0(i);
%     str_temp = str_temp{1};
%     splt_str_temp = strsplit(str_temp,':');
%     t_temp = splt_str_temp(3);
%     tvec = [tvec;str2num(t_temp{1})];
% end
% 
% data_new;
% data_t = [tvec,data_new];
% csvwrite('data_with_time.csv',data_t)
data_t = csvread('data_with_time.csv');
%% Separate Runs
new_run_ind = [1];
tvec = data_t(:,1);
for i = 2:size(data_t(:,1),1)
    if tvec(i) < tvec(i-1)
        new_run_ind = [new_run_ind,i];
    end
end

new_run_ind = [new_run_ind,size(tvec,1)];
new_run_ind
%% Plot appropriate run
% 7,9,10,11,12,14,15,16,20
run_num = 7;
t_ind_low = new_run_ind(run_num);
t_ind_high = new_run_ind(run_num+1)-1;

data = [data_t(t_ind_low:t_ind_high,:)];
% Altitude tracking
figure
% subplot(2,2,1)
plot(data(:,1),data(:,9)/1000)
hold on
plot(linspace(data(1,1),data(end,1),size(data(:,1),1)),ones(size(data(:,1))),'k--')
axis([data(1,1) data(end,1) 0 1.5])
xlabel('time(s)')
ylabel('altitude (m)')
legend('System Response','Desired track')
% X tracking
figure
% subplot(2,2,2)
plot(data(:,1),data(:,7)/1000)
hold on
plot(linspace(data(1,1),data(end,1),size(data(:,1),1)),0*ones(size(data(:,1))),'k--')
axis([data(1,1) data(end,1) -2 2])
xlabel('time(s)')
ylabel('X Position (m)')
legend('System Response','Desired track')
% Y tracking
figure
% subplot(2,2,3)
plot(data(:,1),data(:,8)/1000)
hold on
plot(linspace(data(1,1),data(end,1),size(data(:,1),1)),0*ones(size(data(:,1))),'k--')
axis([data(1,1) data(end,1) -2 2])
xlabel('time(s)')
ylabel('Y position (m)')
legend('System Response','Desired track')

yawref = [0 0;64 0;64 .1;104 .1;104 .5;126,.5;126 1; 159 1;159 1.37;183 1.37;183 1.8;216 1.8;216 2.2;249 2.2;249 2.6;251 2.6;251 3; 263 3;263 3.4;265 3.4;265 3.8;267 3.8;267 4.2;271 4.2;271 4.6;273.5 4.6;273.5 5;336 5;336 5.4;379 5.4;379 5.8;400 5.8 ];
yawref = yawref + [3.5*ones(size(yawref,1),1),zeros(size(yawref,1),1)];
% subplot(2,2,4)
figure
plot(data(:,1),data(:,10))
hold on
plot(yawref(:,1),yawref(:,2),'k--')
xlabel('t(s)')
ylabel('Yaw (radians)')
legend('System Response','Desired track')
% % Spatial movement
% figure
% plot3(data(:,7)/1000,data(:,8)/1000,data(:,9)/1000)
% axis([-2 2 -2 2 0 2])
% grid on