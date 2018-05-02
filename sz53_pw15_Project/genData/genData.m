
t = 0:0.1:2*pi;
%%
x = 20*cos(t)-20;
y = 20*sin(t);
z = [1:length(x)]/10+170;

s = zeros(3,length(z));
for ii = 1:length(x)
    s(1,ii) = x(ii);
    s(2,ii) = y(ii);
    s(3,ii) = z(ii);
end

