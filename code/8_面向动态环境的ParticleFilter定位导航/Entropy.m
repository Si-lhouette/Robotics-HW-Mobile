w = repmat([1],[1,10]);
w = [];
for i = 1:10
    w=[w,rand()];
end
w = wnorm(w);
plot([1:1:10],w,'r.')
grid on
H = cal_H(w)

% k = [0:0.01:1];
% y = [];
% for i = 1:length(k)
%     y = [y,-k(i)*log2(k(i))-(1-k(i))*log2((1-k(i)))];
% end
% plot(k,y)
% grid on

function w = wnorm(win)
    s = sum(win);
    w = win./s;
end

function H = cal_H(win)
    H = 0;
    for i=1:length(win)
        H = H - win(i)*log2(win(i));
    end
end