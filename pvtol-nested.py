#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#このファイルは、Python制御パッケージの基本的な機能を実証することを目的としています。
#AstromとMrurayの平面垂直離着陸（PVTOL）機に対応する、かなり複雑な制御設計と解析を動作します。
#


# pvtol-nested.py - aircraftのスラスタベクトルの内外ループ制御設計


from __future__ import print_function
from matplotlib.pyplot import * # MATLAB プロット関数
from control.matlab import *    # MATLAB-like 関数
import numpy as np

# システムのパラメータ
m = 4;                         # aircraftの質量
J = 0.0475;                    #ピッチ軸周りの慣性
r = 0.25;                      #力の中心までの距離
g = 9.8;                       # 重力定数
c = 0.05;                      # 減衰係数（推定値）


#ダイナミクスの伝達関数
Pi = tf([r], [J, 0, 0]);       # 内側のループ (Roll角度)
Po = tf([1], [m, c, 0]);       # 外側のループ (位置)

# Use state space versions
Pi = tf2ss(Pi);
Po = tf2ss(Po);

#
# 内側のループ制御設計
#

# システムのシンプルなリードコントローラの設計
k = 200;  a = 2;  b = 50;
Ci = k*tf([1, a], [1, b]);		# リード補償
Li = Pi*Ci;

#オープンループのボード線図
figure(1); 
bode(Pi);
show()

# マージンを含めたループ伝達関数のボード線図
figure(2); 
bode(Li);
show()
# ゲインと位相マージンを計算する
#! 実装されていなかった
(gm, pm, wcg, wcp) = margin(Li);
print (gm, pm, wcg, wcp)

# 感度と相補感度関数を計算する
Si = feedback(1, Li);
Ti = Li * Si;

# 仕様が満たされていることを確認する
figure(3);  gangof4(Pi, Ci);
# u1からv1への実際の伝達関数を計算する(see L8.2 notes)
# Hi = Ci*(1-m*g*Pi)/(1+Ci*Pi);
Hi = parallel(feedback(Ci, Pi), -m*g*feedback(Ci*Pi, 1));
show()

figure(4); clf; subplot(221);
bode(Hi);

# 横方向制御システムをここで設計する
a = 0.02; b = 5; K = 2;
Co = -K*tf([1, 0.3], [1, 10]);		# another lead compensator
Lo = -m*g*Po*Co;
show()


figure(5); 
bode(Lo);                       # margin(Lo)
#最後に、実際の外側のループのループゲインと応答を計算する
L = Co*Hi*Po;
S = feedback(1, L);
T = feedback(L, 1);

# 安定性マージンの計算
#! 実装されていなかった
(gm, pm, wgc, wpc) = margin(L); 
print (gm, pm, wgc, wpc)
show()

#! TODO:この数字には何か問題があります。軸の制限が不一致
figure(6); clf; 
bode(L);

# クロスオーバーラインを追加
subplot(211); hold(True);
loglog([1e-4, 1e3], [1, 1], 'k-')

#-90度から始まるように位相反転
bode(L, logspace(-4, 3));
(mag, phase, w) = freqresp(L, logspace(-4, 3));
phase = phase - 360;
subplot(212);
semilogx([1e-4, 1e3], [-180, -180], 'k-')
hold(True);
semilogx(w, np.squeeze(phase), 'b-')
axis([1e-4, 1e3, -360, 0]);
xlabel('Frequency [deg]'); ylabel('Phase [deg]');
# set(gca, 'YTick', [-360, -270, -180, -90, 0]);
# set(gca, 'XTick', [10^-4, 10^-2, 1, 100]);
show()
#
# ナイキスト線図
#
figure(7); clf;
axis([-700, 5300, -3000, 3000]); hold(True);
nyquist(L, (0.0001, 1000));
axis([-700, 5300, -3000, 3000]);

# 展開する領域にボックスを追加する
plot([-400, -400, 200, 200, -400], [-100, 100, 100, -100, -100], 'r-')
show()

# 拡張領域
figure(8); clf; subplot(231); 
axis([-10, 5, -20, 20]); hold(True);
nyquist(L);
axis([-10, 5, -20, 20]);

#色を設定
color = 'b';
show()

# プロットに矢印を追加する
# H1 = L.evalfr(0.4); H2 = L.evalfr(0.41);
# arrow([real(H1), imag(H1)], [real(H2), imag(H2)], AM_normal_arrowsize, \
#  'EdgeColor', color, 'FaceColor', color);

# H1 = freqresp(L, 0.35); H2 = freqresp(L, 0.36);
# arrow([real(H2), -imag(H2)], [real(H1), -imag(H1)], AM_normal_arrowsize, \
#  'EdgeColor', color, 'FaceColor', color);

figure(9); 
(Yvec, Tvec) = step(T, linspace(0, 20));
plot(Tvec.T, Yvec.T); hold(True);

(Yvec, Tvec) = step(Co*S, linspace(0, 20));
plot(Tvec.T, Yvec.T);
show()
figure(10); clf();
(P, Z) = pzmap(T, Plot=True)
print("Closed loop poles and zeros: ", P, Z)
show()
# Gang of Four
figure(11); clf();
gangof4(Hi*Po, Co);

show()

