clear all
close all
clc

fileID = fopen('good_flight_log.txt');
data_matrix = [];
for i =1:1
    C_temp = fgets(fileID);
end

i = 1;
while ~feof(fileID)
    data_temp = strsplit(fgets(fileID),',');
    data_matrix = [data_matrix;data_temp];
    i = i+1;
end
data_new = [];
for i = 1:size(data_matrix,1)
    data_vector_temp = [];
    for j = 2:size(data_matrix,2)
        data_temp = data_matrix(i,j);
        data_vector_temp = [data_vector_temp,str2double(data_temp{1})];
    end
    data_new = [data_new;data_vector_temp];
end

data_new;

tvec = 1:size(data_new,1);
figure
plot(tvec,[data_new(:,9)])
figure
plot(tvec,[data_new(:,5)])
figure
plot(tvec,[data_new(:,15)])


fclose('all')