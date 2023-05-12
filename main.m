clear all; close all; clc

X1=load("data.txt");
x2=load("data.csv");
X=X1;

figure(1);
plot(X(:,1),X(:,2))

plot(X(:,1),X(:,2),'.')

plot(X(:,1),X(:,2),'r.')
plot(X(:,1),X(:,2),'ro')

figure(1);
plot(X(:,1),X(:,2),'.')
hold on;
plot(X(:,1),X(:,2),'ro')
hold off;




