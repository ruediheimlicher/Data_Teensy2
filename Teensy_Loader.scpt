FasdUAS 1.101.10   ��   ��    k             l     ��������  ��  ��        l     	���� 	 O      
  
 k    ~       r        c        l    ����  n        m   	 ��
�� 
ctnr  l   	 ����  I   	�� ��
�� .earsffdralis        afdr   f    ��  ��  ��  ��  ��    m    ��
�� 
ctxt  o      ���� 0 
homeordner        l   ��������  ��  ��        l   ��  ��    4 .set homeordner to alias ((path to me as text))     �   \ s e t   h o m e o r d n e r   t o   a l i a s   ( ( p a t h   t o   m e   a s   t e x t ) )       l   ��������  ��  ��      ! " ! l   ��������  ��  ��   "  # $ # l   �� % &��   % ' !display dialog homeordner as text    & � ' ' B d i s p l a y   d i a l o g   h o m e o r d n e r   a s   t e x t $  ( ) ( l   ��������  ��  ��   )  * + * r     , - , n     . / . 1    ��
�� 
psxp / o    ���� 0 
homeordner   - o      ���� 0 homeordnerpfad   +  0 1 0 I   �� 2��
�� .sysodlogaskr        TEXT 2 c     3 4 3 o    ���� 0 homeordnerpfad   4 m    ��
�� 
ctxt��   1  5 6 5 I   #�� 7��
�� .aevtodocnull  �    alis 7 o    ���� 0 
homeordner  ��   6  8 9 8 l  $ $��������  ��  ��   9  : ; : I  $ )�� <��
�� .sysoexecTEXT���     TEXT < m   $ % = = � > > $ c d   ~ / D o c u m e n t s ;   l s��   ;  ? @ ? I  * 3�� A��
�� .sysoexecTEXT���     TEXT A b   * / B C B m   * + D D � E E  c d   C n   + . F G F 1   , .��
�� 
strq G o   + ,���� 0 homeordnerpfad  ��   @  H I H O   4 l J K J k   8 k L L  M N M I  8 =������
�� .miscactvnull��� ��� null��  ��   N  O P O r   > E Q R Q b   > A S T S m   > ? U U � V V  c d   T o   ? @���� 0 homeordnerpfad   R o      ���� 0 pfad   P  W X W I  F M�� Y��
�� .sysodlogaskr        TEXT Y o   F I���� 0 pfad  ��   X  Z [ Z I  N W�� \��
�� .sysoexecTEXT���     TEXT \ n   N S ] ^ ] 1   Q S��
�� 
strq ^ o   N Q���� 0 pfad  ��   [  _ ` _ r   X c a b a I  X _�� c��
�� .sysoexecTEXT���     TEXT c m   X [ d d � e e  l s  ��   b o      ���� 
0 inhalt   `  f g f l  d d�� h i��   h 8 2set inhalt to do shell script "cd ~/Documents; ls"    i � j j d s e t   i n h a l t   t o   d o   s h e l l   s c r i p t   " c d   ~ / D o c u m e n t s ;   l s " g  k�� k I  d k�� l��
�� .sysodlogaskr        TEXT l o   d g���� 
0 inhalt  ��  ��   K m   4 5 m m�                                                                                      @ alis    j  Macintosh HD               ���H+    �Terminal.app                                                    ɔ��        ����  	                	Utilities     �}�      ɔ��      �  �.  0Macintosh HD:Applications:Utilities:Terminal.app    T e r m i n a l . a p p    M a c i n t o s h   H D  #Applications/Utilities/Terminal.app   / ��   I  n o n r   m v p q p c   m r r s r o   m n���� 0 
homeordner   s m   n q��
�� 
alis q o      ���� 0 openpfad   o  t u t l  w w�� v w��   v  open openpfad    w � x x  o p e n   o p e n p f a d u  y�� y I  w ~�� z��
�� .sysodlogaskr        TEXT z m   w z { { � | |  .��  ��    m      } }�                                                                                  MACS  alis    r  Macintosh HD               ���H+   ��
Finder.app                                                      	_ɚL_        ����  	                CoreServices    �}�      ɚ>O     �� ��  �6  3Macintosh HD:System:Library:CoreServices:Finder.app    
 F i n d e r . a p p    M a c i n t o s h   H D  &System/Library/CoreServices/Finder.app  / ��  ��  ��     ~�� ~ l  � � ����  O   � � � � � k   � � � �  � � � l  � � � � � � I  � �������
�� .miscactvnull��� ��� null��  ��   �   "#!/bin/bash    � � � �    " # ! / b i n / b a s h �  � � � r   � � � � � I  � ��� ���
�� .sysoexecTEXT���     TEXT � m   � � � � � � �  e c h o   $ P A T H��   � o      ���� 
0 dir Dir �  � � � I  � ��� ���
�� .sysodlogaskr        TEXT � o   � ����� 
0 dir Dir��   �  � � � l  � � � � � � I  � ��� ���
�� .sysoexecTEXT���     TEXT � m   � � � � � � �  / b i n / b a s h   c d   . /��   �  & Dir    � � � � 
 &   D i r �  � � � r   � � � � � b   � � � � � m   � � � � � � � $ # ! / b i n / b a s h   
 	 c d     � o   � ����� 0 homeordnerpfad   � o      ���� 0 cmd   �  � � � I  � ��� ���
�� .sysodlogaskr        TEXT � o   � ����� 0 cmd  ��   �  � � � l  � ��� � ���   �  do shell script cmd    � � � � & d o   s h e l l   s c r i p t   c m d �  ��� � l  � ��� � ���   � O Ido shell script "teensy_loader_cli -mmcu=at90usb1286 -w -v TWI_Slave.hex"    � � � � � d o   s h e l l   s c r i p t   " t e e n s y _ l o a d e r _ c l i   - m m c u = a t 9 0 u s b 1 2 8 6   - w   - v   T W I _ S l a v e . h e x "��   � m   � � � ��                                                                                      @ alis    j  Macintosh HD               ���H+    �Terminal.app                                                    ɔ��        ����  	                	Utilities     �}�      ɔ��      �  �.  0Macintosh HD:Applications:Utilities:Terminal.app    T e r m i n a l . a p p    M a c i n t o s h   H D  #Applications/Utilities/Terminal.app   / ��  ��  ��  ��       �� � � � � ���   � ��������
�� .aevtoappnull  �   � ****�� 0 
homeordner  �� 0 homeordnerpfad  �� 0 pfad   � �� ����� � ���
�� .aevtoappnull  �   � **** � k     � � �   � �  ~����  ��  ��   �   �  }���������������� =�� D�� m�� U�� d������ { ��� � ���
�� .earsffdralis        afdr
�� 
ctnr
�� 
ctxt�� 0 
homeordner  
�� 
psxp�� 0 homeordnerpfad  
�� .sysodlogaskr        TEXT
�� .aevtodocnull  �    alis
�� .sysoexecTEXT���     TEXT
�� 
strq
�� .miscactvnull��� ��� null�� 0 pfad  �� 
0 inhalt  
�� 
alis�� 0 openpfad  �� 
0 dir Dir�� 0 cmd  �� �� |)j �,�&E�O��,E�O��&j O�j O�j 
O���,%j 
O� 5*j O��%E` O_ j O_ �,j 
Oa j 
E` O_ j UO�a &E` Oa j UO� 7*j Oa j 
E` O_ j Oa j 
Oa �%E` O_ j OPU � � � � � M a c i n t o s h   H D : U s e r s : r u e d i h e i m l i c h e r : D o c u m e n t s : E l e k t r o n i k : C N C - P r o j e k t : C N C _ S l a v e : C N C _ U S B : C N C _ U S B _ 2 : � � � � � / U s e r s / r u e d i h e i m l i c h e r / D o c u m e n t s / E l e k t r o n i k / C N C - P r o j e k t / C N C _ S l a v e / C N C _ U S B / C N C _ U S B _ 2 / � � � � � c d   / U s e r s / r u e d i h e i m l i c h e r / D o c u m e n t s / E l e k t r o n i k / C N C - P r o j e k t / C N C _ S l a v e / C N C _ U S B / C N C _ U S B _ 2 /ascr  ��ޭ