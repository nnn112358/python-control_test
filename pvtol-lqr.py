#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pvtol_lqr.m - LQR design for vectored thrust aircraft
# RMM, 14 Jan 03
#

#このファイルは、Astrom and Mruray第5章の平面垂直離着陸（PVTOL）航空機の例を使用して、
#LQRベースの設計上の問題をPythonコントロールパッケージの基本機を使って処理します。
#

from numpy import *             # NumPy関数
from matplotlib.pyplot import * # MATLAB プロット関数
from control.matlab import *    # MATLAB-like 関数

#
# System dynamics
#
# 状態空間形式のPVTOLシステムのダイナミクス
#

# システムのパラメータ
m = 4;                         # aircraftの質量
J = 0.0475;                    #ピッチ軸周りの慣性
r = 0.25;                      #力の中心までの距離
g = 9.8;                       # 重力定数
c = 0.05;                      # 減衰係数（推定値）


#  dynamicsの状態空間
xe = [0, 0, 0, 0, 0, 0];        # 平衡点
ue = [0, m*g];                  # (これらはリストであり行列ではないことに注意してください)

# Dynamics 行列 (*が動作するように行列型を使用する)
A = matrix(
    [[ 0,    0,    0,    1,    0,    0],
     [ 0,    0,    0,    0,    1,    0],
     [ 0,    0,    0,    0,    0,    1],
     [ 0, 0, (-ue[0]*sin(xe[2]) - ue[1]*cos(xe[2]))/m, -c/m, 0, 0],
     [ 0, 0, (ue[0]*cos(xe[2]) - ue[1]*sin(xe[2]))/m, 0, -c/m, 0],
     [ 0,    0,    0,    0,    0,    0 ]])

# Input 行列
B = matrix(
    [[0, 0], [0, 0], [0, 0],
     [cos(xe[2])/m, -sin(xe[2])/m],
     [sin(xe[2])/m,  cos(xe[2])/m],
     [r/J, 0]])

# Output 行列 
C = matrix([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]])
D = matrix([[0, 0], [0, 0]])

#
#　xy位置のstepに対応する入力と出力を構築する
#ベクトルxdおよびydは、システムの所望の平衡状態である状態に対応する。
#行列CxおよびCyは、対応する出力である。
#
# これらのベクトルは、閉ループシステムのダイナミクスを次のように計算することで使用される。

#
#	xdot = Ax + B u		=>	xdot = (A-BK)x + K xd
#         u = -K(x - xd)		   y = Cx
#
# 閉ループ動特性は、入力ベクトルとしてK * xdを用いて「step」コマンドを使用してシミュレートすることができる
# （「入力」は単位サイズであると仮定し、xdは所望の定常状態に対応する）。
#

xd = matrix([[1], [0], [0], [0], [0], [0]]); 
yd = matrix([[0], [1], [0], [0], [0], [0]]);  

#
# 関連するダイナミクスを抽出してSISOライブラリで使用する
#
# 現在のpython-controlライブラリはSISO転送関数しかサポートしていないので、
# 元のMATLABコードの一部を修正してSISOシステムを抽出する必要があります。
# これを行うために、横（x）および縦（y）ダイナミクスに関連する状態からなるように、
# 「lat」および「alt」インデックスベクトルを定義します。
#

# 私たちが状態変数

lat = (0,2,3,5);
alt = (1,4);

#分離されたダイナミックス
Ax = (A[lat, :])[:, lat];       #!なぜこのようにしなければならないのか分からない!
Bx = B[lat, 0]; Cx = C[0, lat]; Dx = D[0, 0];

Ay = (A[alt, :])[:, alt];       #!なぜこのようにしなければならないのか分からない!
By = B[alt, 1]; Cy = C[1, alt]; Dy = D[1, 1];

#  plotラベル
clf(); 
suptitle("LQR controllers for vectored thrust aircraft (pvtol-lqr)")

#
# LQR design
#

# 対角行列の重み付け
Qx1 = diag([1, 1, 1, 1, 1, 1]);
Qu1a = diag([1, 1]);
(K, X, E) = lqr(A, B, Qx1, Qu1a); K1a = matrix(K);

# ループを閉じる: xdot = Ax - B K (x-xd)
# Note: python-controlでは、この入力を一度に行う必要があります
#　H1a = ss(A-B*K1a, B*K1a*concatenate((xd, yd), axis=1), C, D)　
# (T, Y) = step(H1a, T=linspace(0,10,100));

# 最初の入力に対するステップ応答
H1ax = ss(Ax - Bx*K1a[0,lat], Bx*K1a[0,lat]*xd[lat,:], Cx, Dx);
(Yx, Tx) = step(H1ax, T=linspace(0,10,100));

# 第2入力に対するステップ応答
H1ay = ss(Ay - By*K1a[1,alt], By*K1a[1,alt]*yd[alt,:], Cy, Dy);
(Yy, Ty) = step(H1ay, T=linspace(0,10,100));

subplot(221); title("Identity weights")
# plot(T, Y[:,1, 1], '-', T, Y[:,2, 2], '--'); hold(True);
plot(Tx.T, Yx.T, '-', Ty.T, Yy.T, '--'); hold(True);
plot([0, 10], [1, 1], 'k-'); hold(True);

axis([0, 10, -0.1, 1.4]); 
ylabel('position');
legend(('x', 'y'), loc='lower right');

# 異なる入力重みを見る
Qu1a = diag([1, 1]); (K1a, X, E) = lqr(A, B, Qx1, Qu1a);
H1ax = ss(Ax - Bx*K1a[0,lat], Bx*K1a[0,lat]*xd[lat,:], Cx, Dx);

Qu1b = (40**2)*diag([1, 1]); (K1b, X, E) = lqr(A, B, Qx1, Qu1b);
H1bx = ss(Ax - Bx*K1b[0,lat], Bx*K1b[0,lat]*xd[lat,:],Cx, Dx);

Qu1c = (200**2)*diag([1, 1]); (K1c, X, E) = lqr(A, B, Qx1, Qu1c);
H1cx = ss(Ax - Bx*K1c[0,lat], Bx*K1c[0,lat]*xd[lat,:],Cx, Dx);

[Y1, T1] = step(H1ax, T=linspace(0,10,100));
[Y2, T2] = step(H1bx, T=linspace(0,10,100));
[Y3, T3] = step(H1cx, T=linspace(0,10,100));

subplot(222); title("Effect of input weights")
plot(T1.T, Y1.T, 'b-'); hold(True);
plot(T2.T, Y2.T, 'b-'); hold(True);
plot(T3.T, Y3.T, 'b-'); hold(True);
plot([0 ,10], [1, 1], 'k-'); hold(True);

axis([0, 10, -0.1, 1.4]); 

# arcarrow([1.3, 0.8], [5, 0.45], -6);
text(5.3, 0.4, 'rho');

# 出力重み付け - 出力を使用するようにQxを変更する
Qx2 = C.T * C;
Qu2 = 0.1 * diag([1, 1]);
(K, X, E) = lqr(A, B, Qx2, Qu2); K2 = matrix(K)

H2x = ss(Ax - Bx*K2[0,lat], Bx*K2[0,lat]*xd[lat,:], Cx, Dx);
H2y = ss(Ay - By*K2[1,alt], By*K2[1,alt]*yd[alt,:], Cy, Dy);

subplot(223); title("Output weighting")
[Y2x, T2x] = step(H2x, T=linspace(0,10,100));
[Y2y, T2y] = step(H2y, T=linspace(0,10,100));
plot(T2x.T, Y2x.T, T2y.T, Y2y.T)
ylabel('position');
xlabel('time'); ylabel('position');
legend(('x', 'y'), loc='lower right');

#
# 　物理的に動機付けされた重み付け
#
# xで1 cmの誤差、yで10 cmの誤差で決定する。
# 角度を5度以下に調整して調整する。
# 効率の低下により、サイドの力にはペナルティを課す。
#

Qx3 = diag([100, 10, 2*pi/5, 0, 0, 0]);
Qu3 = 0.1 * diag([1, 10]);
(K, X, E) = lqr(A, B, Qx3, Qu3); K3 = matrix(K);

H3x = ss(Ax - Bx*K3[0,lat], Bx*K3[0,lat]*xd[lat,:], Cx, Dx);
H3y = ss(Ay - By*K3[1,alt], By*K3[1,alt]*yd[alt,:], Cy, Dy);
subplot(224)
# step(H3x, H3y, 10);
[Y3x, T3x] = step(H3x, T=linspace(0,10,100));
[Y3y, T3y] = step(H3y, T=linspace(0,10,100));
plot(T3x.T, Y3x.T, T3y.T, Y3y.T)
title("Physically motivated weights")
xlabel('time'); 
legend(('x', 'y'), loc='lower right');

show()
