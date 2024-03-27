close all
clc
clear

%% 车辆属性赋值
properties.nMin = 600;     %发动机最低转速r/min
properties.nMax = 4000;    %发动机最高转速r/min
properties.mTotal = 3880;  %总质量kg
properties.mCar = 1800;    %整车整备质量kg
properties.nLoad = 2000;   %装载质量kg
properties.r = 0.367;      %车轮半径m
properties.yitaT = 0.85;   %传动系机械效率
properties.f = 0.013;      %滚动阻力系数
properties.CdA = 2.77;     %空气阻力系数*迎风面积m^2
properties.i0 = 5.83;      %主减速器传动比
properties.If = 0.218;     %飞轮转动惯量kg*m^2
properties.Iw1 = 1.798;    %二前轮转动惯量kg*m^2
properties.Iw2 = 3.598;    %四后轮转动惯量kg*m^2
properties.L = 3.2;        %轴距m
properties.a = 1.947;      %质心至前轴距离（满载）m
properties.b = properties.L - properties.a; %质心至后轴距离（满载）m
properties.hg = 0.9;       %质心高m
properties.gearbox = 5;                        %使用五档变速器
properties.ig = [6.09,3.09,1.71,1.00,NaN       %变速器传动比(四档变速器)
                 5.56,2.769,1.644,1.00,0.793]; %变速器传动比(五档变速器)
properties.ig = properties.ig(properties.gearbox-3,:);
properties.p = [-3.8445,40.874,-165.44,295.27,-19.313]; %外特性Tq-n曲线拟合参数
properties.Tq_n = @(n)polyval(properties.p,n/1000);


%% Question1 绘制驱动力-行驶阻力平衡图
%%%% 计算部分 %%%%
Ft = @(n,i)properties.Tq_n(n) * properties.ig(i) * properties.i0 * properties.yitaT / properties.r;
ua = @(n,i)0.377*properties.r.*n/(properties.ig(i).*properties.i0);
Fw = @(ua)properties.CdA .* ua.^2 / 21.15;
Ff = properties.mTotal * 9.8 * properties.f;
for n = properties.nMin : properties.nMax
    for i = 1 : properties.gearbox            %i:变速器挡位
        %1. 计算每一个n与i下的驱动力Ft
        FtM(i,n-properties.nMin+1) = Ft(n,i);
        %1. 计算每一个n与i下的车速ua
        uaM(i,n-properties.nMin+1) = ua(n,i);
    end
end
%3. 计算每个发动机转速下的外界阻力
uaMin = min(min(uaM));
uaMax = max(max(uaM));
FwM = properties.CdA * (uaMin:uaMax).^2 / 21.15;
Fw_Ff = FwM + Ff;

%%% 绘图部分 %%%%
f1 = figure;
hold on
plot(uaMin:uaMax,Fw_Ff); %Ft+Fw - ua
for i = 1 : properties.gearbox
    plot(uaM(i,:),FtM(i,:));
end
title([num2str(properties.gearbox),'挡变速器的驱动力-行驶阻力平衡图'])
xlabel('ua/(km/h)')
ylabel('Ft/N, (Ft+Fw)/N')
legend('Ft+Fw','Ft1','Ft2','Ft3','Ft4','Ft5');

%% Question2 求汽车最高速度、最大爬坡度以及克服该坡度时的附着率
%%%% 求汽车最高速度 %%%%
%Key:由Q1可知最高车速时驱动力等于行驶阻力
dF =@(n)Ft(n,5) - (Fw(ua(n,5)) + Ff); %驱动力与行驶阻力之差
%采用二分法求函数df的零点
n1 = properties.nMin;
n2 = properties.nMax;
nAve = 0.5*(n1 + n2);
while dF(nAve) > 1e-5
    nAve = 0.5*(n1 + n2);
    if dF(nAve) < 0
        n2 = nAve;
    else
        n1 = nAve;
    end
end
uaMax = ua(nAve,5); %最高车速
disp(['汽车最高车速为',num2str(uaMax)]);

%%%% 求汽车的最大爬坡度 %%%%
%Key:汽车在克服Ff+Fw后的余力均用于克服爬坡阻力
Fi = @(n,gearboxi)Ft(n,gearboxi) - Fw(ua(n,gearboxi)) - Ff;
G =  properties.mTotal * 9.8;
alpha = @(n,gearboxi)asin(Fi(n,gearboxi)/G);
i = @(n,gearboxi)tan(alpha(n,gearboxi));
for n = properties.nMin : properties.nMax
    for gearboxi = 1 : properties.gearbox
        iM(gearboxi,n-properties.nMin+1) = i(n,gearboxi);
    end
end

%%%% 绘图部分 %%%%
f2 = figure;
hold on
for gearboxi = 1 : properties.gearbox
    plot(uaM(gearboxi,:),iM(gearboxi,:));
end
title([num2str(properties.gearbox),'挡变速器的爬坡度图'])
xlabel('ua/(km/h)')
ylabel('i/(%)')
legend('1挡','2挡','3挡','4挡','5挡');

%%%% 采用三点法求i的极值 %%%%
%可知当车辆处于一档时存在爬坡度极值
nLeft = properties.nMin;
nRight = properties.nMax;
midL = (2*nLeft + nRight)/3;
midR = (nLeft + 2*nRight)/3;
iLeft = i(midL,1);
iRight = i(midR,1);
while abs(iLeft - iRight) < 1e-5
    midL = (2*nLeft + nRight)/3;
    midR = (nLeft + 2*nRight)/3;
    iLeft = i(midL,1);
    iRight = i(midR,1);
    if iLeft > iRight
        nRight =  midR;
    else
        nLeft =  midL;
    end
end
iMax = i(0.5*(nLeft+nRight),1);
disp(['汽车最大爬坡度为',num2str(iMax)]);

%%%% 求解iMax坡度时的附着率 %%%%
%假设车辆为后驱，iMax坡度时加速阻力为0
q = iMax;
Cphy2 = q/((properties.a/properties.L) + q*properties.hg/properties.L);
disp(['后驱时相应附着率为',num2str(Cphy2)]);

%假设车辆为前驱，iMax坡度时加速阻力为0
Cphy1 = q/((properties.b/properties.L) - q*properties.hg/properties.L);
disp(['前驱时相应附着率为',num2str(Cphy1)]);
%% Question3 绘制加速度倒数曲线，使用计算机求解汽车用2挡起步加速行驶至70km/h所需时间
%%%% 获得t-n的函数关系 %%%%
kesi = @(gearboxi) 1 + ((properties.Iw1+properties.Iw2)/properties.mTotal*properties.r^2) + ((properties.If*properties.yitaT*properties.ig(gearboxi)^2*properties.i0^2)/(properties.mTotal*properties.r^2));
a = @(n,gearboxi)3.6*(Ft(n,gearboxi) - Fw(ua(n,gearboxi)) - Ff)/(kesi(gearboxi)*properties.mTotal);
ar = @(n,gearboxi)1/a(n,gearboxi);
for n = properties.nMin : properties.nMax
    for gearboxi = 1 : properties.gearbox - 1
        arM(gearboxi,n-properties.nMin+1) = ar(n,gearboxi);
    end
end

%%%% 绘图 %%%%
f3 = figure;
hold on
for gearboxi = 1 : properties.gearbox - 1
    plot(uaM(gearboxi,:),arM(gearboxi,:));
end
title([num2str(properties.gearbox),'挡变速器的汽车加速度倒数曲线'])
xlabel('ua/(km/h)')
ylabel('1/a')
legend('1挡','2挡','3挡','4挡');

%%%% 数值积分 %%%%
d = 0.01;
n = @(ua,gearboxi) ua*properties.i0*properties.ig(gearboxi)/(0.377*properties.r);
a_result = []; %各ui下的加速度倒数
for ui = ua(properties.nMin,2):d:70
    n_list = [n(ui,2),n(ui,3),n(ui,4)];                        %ui速度下的各档转速
    a_list = [a(n(ui,2),2),a(n(ui,3),3),a(n(ui,4),4)];         %ui速度下的格挡加速度倒数
    %判断ui速度下转速是否超标
    for gearboxi = 1:3
        if n_list(gearboxi) < properties.nMin || n_list(gearboxi) > properties.nMax 
           a_list(gearboxi) = -inf;
        end
    end
    a_result = [a_result max(a_list)];
end

t = 0;
resultN = size(a_result,2);
for resulti = 1 : resultN - 1
ti = d/a_result(resulti);
t = t + ti;
end
disp(['加速所需时间为',num2str(t)]);


