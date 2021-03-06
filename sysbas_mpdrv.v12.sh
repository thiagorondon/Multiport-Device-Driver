#!/bin/sh

CRCsum="2995297776"
MD5="bd68b001390ed7d41c882ee2a03a47e3"
TMPROOT=${TMPDIR:=/tmp}

label="Systembase PCI/PCIe device drvier installer"
script="./Install"
scriptargs=""
targetdir="sysbas_mpdrv.v12"
filesizes="25837"

print_cmd_arg=""
if type printf > /dev/null; then
    print_cmd="printf"
elif test -x /usr/ucb/echo; then
    print_cmd="/usr/ucb/echo"
else
    print_cmd="echo"
fi

unset CDPATH

MS_Printf()
{
    $print_cmd $print_cmd_arg "$1"
}

MS_Progress()
{
    while read a; do
	MS_Printf " "
    done
}

MS_diskspace()
{
	(
	if test -d /usr/xpg4/bin; then
		PATH=/usr/xpg4/bin:$PATH
	fi
	df -kP "$1" | tail -1 | awk '{print $4}'
	)
}

MS_dd()
{
    blocks=`expr $3 / 1024`
    bytes=`expr $3 % 1024`
    dd if="$1" ibs=$2 skip=1 obs=1024 conv=sync 2> /dev/null | \
    { test $blocks -gt 0 && dd ibs=1024 obs=1024 count=$blocks ; \
      test $bytes  -gt 0 && dd ibs=1 obs=1024 count=$bytes ; } 2> /dev/null
}

MS_Help()
{
    cat << EOH >&2
* Release note *

        Before installation,
        make sure system with c compiler and kerenl development package.

        Ver 1.00
                Test RedHat 9.0     ( kernel 2.4.20 )
                Test Fedora core  1 ( kernel 2.4.22 )
                Test Fedora core  4 ( kernel 2.6.11 )
                Test Fedora core  5 ( kernel 2.6.15 )
                Test Fedora core  6 ( kernel 2.6.18 )
                Test Fedora core  7 ( kernel 2.6.21 )
                Test Fedora core  8 ( kernel 2.6.23 )
                Test Fedora core  9 ( kernel 2.6.25 )
                Test Fedora core 10 ( kernel 2.6.27 )
                Test Fedora core 11 ( kernel 2.6.29 )
                Test Fedora core 12 ( kernel 2.6.31 )
                Test Redhat Enterprise Linux 5 ( kernel 2.6.18-8.el5 )
	Ver 1.1
                Test Cent OS 5.4 ( kernel 2.6.18-164.el5 )
	Ver 1.2
                Test Ubuntu 9.10 ( kernel 2.6.31-14 )
                Test OpenSuse 11.2 ( kernel 2.6.31-5 )
EOH
}

MS_Check()
{
    OLD_PATH="$PATH"
    PATH=${GUESS_MD5_PATH:-"$OLD_PATH:/bin:/usr/bin:/sbin:/usr/local/ssl/bin:/usr/local/bin:/opt/openssl/bin"}
	MD5_ARG=""
    MD5_PATH=`exec <&- 2>&-; which md5sum || type md5sum`
    test -x "$MD5_PATH" || MD5_PATH=`exec <&- 2>&-; which md5 || type md5`
	test -x "$MD5_PATH" || MD5_PATH=`exec <&- 2>&-; which digest || type digest`
    PATH="$OLD_PATH"

    offset=`head -n 298 "$1" | wc -c | tr -d " "`
    verb=$2
    i=1
    for s in $filesizes
    do
		crc=`echo $CRCsum | cut -d" " -f$i`
		if test -x "$MD5_PATH"; then
			if test `basename $MD5_PATH` = digest; then
				MD5_ARG="-a md5"
			fi
			md5=`echo $MD5 | cut -d" " -f$i`
			if test $md5 = "00000000000000000000000000000000"; then
				test x$verb = xy && echo " $1 does not contain an embedded MD5 checksum." >&2
			else
				md5sum=`MS_dd "$1" $offset $s | eval "$MD5_PATH $MD5_ARG" | cut -b-32`;
				if test "$md5sum" != "$md5"; then
					echo "Error in MD5 checksums: $md5sum is different from $md5" >&2
					exit 2
				else
					test x$verb = xy && MS_Printf " MD5 checksums are OK." >&2
				fi
				crc="0000000000"; verb=n
			fi
		fi
		if test $crc = "0000000000"; then
			test x$verb = xy && echo " $1 does not contain a CRC checksum." >&2
		else
			sum1=`MS_dd "$1" $offset $s | CMD_ENV=xpg4 cksum | awk '{print $1}'`
			if test "$sum1" = "$crc"; then
				test x$verb = xy && MS_Printf " CRC checksums are OK." >&2
			else
				echo "Error in checksums: $sum1 is different from $crc"
				exit 2;
			fi
		fi
		i=`expr $i + 1`
		offset=`expr $offset + $s`
    done
}

UnTAR()
{
    tar $1vf - 2>&1 || { echo Install failed. > /dev/tty; kill -15 $$; }
}

finish=true
xterm_loop=
nox11=n
copy=none
ownership=y
verbose=n

initargs="$@"

while true
do
    case "$1" in
    -h | --help)
	MS_Help
	exit 0
	;;
    --lsm)
cat << EOLSM
No LSM.
EOLSM
	exit 0
	;;
	--tar)
	offset=`head -n 298 "$0" | wc -c | tr -d " "`
	arg1="$2"
	shift 2
	for s in $filesizes
	do
	    MS_dd "$0" $offset $s | eval "bzip2 -d" | tar "$arg1" - $*
	    offset=`expr $offset + $s`
	done
	exit 0
	;;
    -*)
	echo Unrecognized flag : "$1" >&2
	MS_Help
	exit 1
	;;
    *)
	break ;;
    esac
done

case "$copy" in
copy)
    tmpdir=$TMPROOT/makeself.$RANDOM.`date +"%y%m%d%H%M%S"`.$$
    mkdir "$tmpdir" || {
	echo "Could not create temporary directory $tmpdir" >&2
	exit 1
    }
    SCRIPT_COPY="$tmpdir/makeself"
    echo "Copying to a temporary location..." >&2
    cp "$0" "$SCRIPT_COPY"
    chmod +x "$SCRIPT_COPY"
    cd "$TMPROOT"
    exec "$SCRIPT_COPY" --phase2 -- $initargs
    ;;
phase2)
    finish="$finish ; rm -rf `dirname $0`"
    ;;
esac

if test "$nox11" = "n"; then
    if tty -s; then                 # Do we have a terminal?
	:
    else
        if test x"$DISPLAY" != x -a x"$xterm_loop" = x; then  # No, but do we have X?
            if xset q > /dev/null 2>&1; then # Check for valid DISPLAY variable
                GUESS_XTERMS="xterm rxvt dtterm eterm Eterm kvt konsole aterm"
                for a in $GUESS_XTERMS; do
                    if type $a >/dev/null 2>&1; then
                        XTERM=$a
                        break
                    fi
                done
                chmod a+x $0 || echo Please add execution rights on $0
                if test `echo "$0" | cut -c1` = "/"; then # Spawn a terminal!
                    exec $XTERM -title "$label" -e "$0" --xwin "$initargs"
                else
                    exec $XTERM -title "$label" -e "./$0" --xwin "$initargs"
                fi
            fi
        fi
    fi
fi

if test "$targetdir" = "."; then
    tmpdir="."
else
    tmpdir="$TMPROOT/selfgz$$$RANDOM"
    dashp=""
    mkdir $dashp $tmpdir || {
	echo 'Cannot create target directory' $tmpdir >&2
	echo 'You should try option --target OtherDirectory' >&2
	eval $finish
	exit 1
    }
fi

location="`pwd`"
if test x$SETUP_NOCHECK != x1; then
    MS_Check "$0"
fi
offset=`head -n 298 "$0" | wc -c | tr -d " "`

if test x"$verbose" = xy; then
	MS_Printf "About to extract 160 KB in $tmpdir ... Proceed ? [Y/n] "
	read yn
	if test x"$yn" = xn; then
		eval $finish; exit 1
	fi
fi

res=3
trap 'echo Check System. >&2; cd $TMPROOT; /bin/rm -rf $tmpdir; eval $finish; exit 15' 1 2 3 15

leftspace=`MS_diskspace $tmpdir`
if test $leftspace -lt 160; then
    echo
    echo "Not enough space left in "`dirname $tmpdir`" ($leftspace KB) to decompress $0 (160 KB)" >&2
    echo "Consider setting TMPDIR to a directory with more free space."
    eval $finish; exit 1
fi

for s in $filesizes
do
    if MS_dd "$0" $offset $s | eval "bzip2 -d" | ( cd "$tmpdir"; UnTAR x ) | MS_Progress; then
		if test x"$ownership" = xy; then
			(PATH=/usr/xpg4/bin:$PATH; cd "$tmpdir"; chown -R `id -u` .;  chgrp -R `id -g` .)
		fi
    else
		echo
		echo "Unable to decompress $0" >&2
		eval $finish; exit 1
    fi
    offset=`expr $offset + $s`
done
echo
cp -rf $tmpdir/sysbas_multiport/ ./

cd "$tmpdir"
res=0
if test x"$script" != x; then
    if test x"$verbose" = xy; then
		MS_Printf "OK to execute: $script $scriptargs $* ? [Y/n] "
		read yn
		if test x"$yn" = x -o x"$yn" = xy -o x"$yn" = xY; then
			eval $script $scriptargs $*; res=$?;
		fi
    else
		eval $script $scriptargs $*; res=$?
    fi
    if test $res -ne 0; then
		test x"$verbose" = xy && echo "The program '$script' returned an error code ($res)" >&2
    fi
fi
    cd $TMPROOT
    /bin/rm -rf $tmpdir

eval $finish; exit $res
BZh91AY&SY��x� >������������������pa ��
H`�r��:�ͼ�  �7������( S}o���ë�8�]��^����޷���w���<�[a����q�{����}ٮ�w3麶{}���uj�6S�]{�|�y��Pm �q�ۇ��=��%ޯH77)����o�zt��SUN��g��ﻸ�g�.�}����]d����z���m��ң����}�n9��{[�ww�W{��ȯ��u\�N�'�7�sۭ�Y�;��{��-����덱�jλ��جr\5�^ɽup�7W]�V�.m4eK���t��n�ve�u��y�����\�[T-�*L�w\�lj�k���T����h�j�ӝN��w6�+.���hV+��v��=��[��D F�4�4�� M&j�T�5O�O�OM���iA��~��F�� ��SA	�����T��d2(ޔ��{T��=#M����    $D	���i4�ɩ����F��������d����C@h  �	4�M�0#%<Tߩ�L)������M=#G�h=�4�M4Ѡ     �$&�&&��	0���F�J����eM�M��i�i�yA��� @4�� @�`5��*{���OSF�Q⇨h�  �  �  �tx)o��P��>��]��O����q,������V�ai�\�2�����/�<��" �h�8`�y��M�kU|W��-�����q6���ی����X�K-pJ�.3i	�� ���~s����7���o���x�Q�%����cv5dq��І��-SdTl|yY���X�H��{��%�-"��⾧p}.�0��E,@���������a_j	H�ģDa��@9�'��p~E�!!�#B`�%���
"Г��Ï�� !Z��T� ��Hu�'���5������� $ۊ�E�� ��k�?w���O��J	��A5`��_cM���)��'d���BO�?X�]��`T��wvwUP /���w�|}7��j�o=�4��"M��U���:���i��^C�4���ahq�(��ߝ��B����Q��Q��B�����"�������?��AV0"'���DP �,R�I����
TXV�TUd8�*�dR ���EY �1q@�7�"�Q~�JL5� ��+X�g��KW�*�V�b��,E��`��F0��P`�H�A �cA��E�
�,�f�FM����*�%h���F(�¿��$Q Ȃ@F����b-�%���1��=00��5c���c!T'�d�Ũ2Dd;�艪%�J^@ �I_ⶩ��V��6���<{��dW1�t��꓅C�$��a�ٺ孶9�8��:��
�������c-�K�ư���&�5fVt���U�ͺ�y���b1����/b�ae�L��}�;��>h������R����X�^lc=;��w��������X�4.I�A~���)$=l�>��FH� �0b�P��ْ́�3h�@EX��E�T�>JfCV�"���"Ȉ�D���,DbFQ����0TR"(��(����`XȢ�*�"�0DNp)d�1y[V# �4�7�QX(��*�cA`�(��X�����!PI����J5��Tc((,�X�X�cE�DE�4�dH�A�$X,DAb��Հ���0h�(����(�UUDT`�dU�@Ubb*����DD���,F" 0�H��XH*�A��P�<��A6�V)(�,���Q	"(�"�D�� �H���0 =�sM��������)�~���9Z���+E!���(g���S��5�&"�ք�1;��uF�j"b"1 �c1�N�NI���X"�!�ݷhK�o����1���s�i��L�3v��;�������.A�Ǟ���^��)����4MR�}�pXG!�9K�ì gD���9#�iN[/��ï7'ёQ�l�)m�Hܤn̚��B�R��-�`�3��1���q�*�WT���)�R�T/Lcg0��Y8i�H죎W�,,1�8�l�0��,�i��ZaT�M�4Yn1��"
��gU��,�o��K(��I[�m�xs��\k��2�m�W�-CC�]��T]�Y��5E���,�ay�X�}%�l� �y�0�k&i��]ы��iN΃$C�'r2z��D��������bQַ�wx���٭�6!��$P�������:ј��t���
����,�+ɩ�ʬ�t�wT�/,�E���9h#9Z���Qb;��=�[Xa��b�Vj�IAgN\�31X

�TL4U�3�8�HU���T�;�p&sg��g]��A�Ib�{�w�����������Q$Mv�A� �,��	�9�ӧ'�����-�q*�L�`�.b���T�T�Ӳg.1+�s&��REޕ���*���WXf�&%�
E���Ba�h(�;x�Pq���R�2��edv�����E�%�E�TAE�`�MhLq��0�b�l�1�YD��l��.�ʒ�[fqeh�R�fY�3��,^�R)Ԑ2�EV�Em���Z��$�b,X���,Y���en���&R�ll2��%@������\˭��<�+�=��&�`� �Al*��z��C��s�<Y'�L m�w�:�>'����C�s<�%݄�%#ӟWg��;��� z"'�/����Mo����O�G�GN5������*rC>��~�M�ߕ�<��^�a�T˰ҿ2�C��T4�d0��(�P��q/�׬*�#z]�O@UB��`2��W���3u8I��kf���,%t��Aǜ�̫M�j=-�[��>]�=�erbo2m��!qi2��m�����<�T�4"�E����\vÎ���i���1�C�9 �L�v�Щ�5�Ȥ��a=�Β8u�� W�߸߶���m���{^9?��䙓����)FH��X#"�H(
��AF��Pi�~'�}o�<��~��g>������ٷk>�RYK��vc(�/��c�?�J��+7C3���^��N�
#�\Z�@,��Ϗ�j�46/nkQݢ�HCIӥ����L�mD~�0�(�)TN��V�'�|�]�p}qm���n�a'$6Z�C�����+������@�픙b��r�`
>b�Æ����&�?֐T�HJ!���2��:�T�����%e"'B�� w[��bP��F����^"����L����U��SªuR��K_UU�Z媪�-]�ӑUr=x��U�UTUUt]�*��UUUQUUV���UUqJE�w,���!6B�E XL�F)&���!�Ha����a�2�(�zI�e�/����xG�������}]� �7��?��;����{�S����ϼ}/���?�����?G���ϋ����������h���J��0X?�T��P,�>�ɻ�:~Ó�������y>捂��f|?((~�������**|��7���t~��G���>ɍ�E?�G(���:*�u5�/�D��lQuk�Ŭ��=^&�\y(}�Y! 9?w�)?�� ��b\�Q"CN�vWx:=C�m��h�;BN����t�&L�� g�Z �٨�~�vg��(���C���Ad��o	���+��M�@y���h0~���N��5�di			#�����n]���ɰJ/�7��(b��Z6j���M�cL"	շ��<�֏Vw�&�Z�g��H%�u/챳!UY/����A|A��Ǌ!ԡ|-���,�0R˘F��kVK|�U������UD�r�f �6陭Ԋ|��#��3@��Ȩ��e�'�����	\Fk� �翅\��˔��q*�ց���M�_g�Ȇ����4��O�}����Ka3_��&���x�VL�r���#3D��d��G	(�<�K�|�՜��,���=oD� ]�/�Gbb�X�����b+3=$Pr�AF+�����T���C/������m֝���d><��pE��E����Pm�"�)ᱠw˿j�%Zt���:=�󚛪R�t�pU�Bi�RypR��O�����h3<��,��H��&بq	� 8��(�)4 �4�XCSE$����Q8$8&�N��F�"�|�c ��_��>���r>�����wP<� ��P��w�rxO?�����&���0q��exb��Z>2p�FM!7P��P����>�<"�%�ə��kZ�Püꧺꭃ4������r3�n��3�1/@��~��ff��n����zVt��lo�|�|ۨ���*�g����K�z��~Ѱ��$LxsyYR���ʙe���ږxA�
�B��,��=�����"����>L����pռ���Z9��驙��4�hy�("�DH�F��`8�B4�P�^3��8C�ڙ�Ճ��݃ǉ4�ѩ��g�J* ��I#�Ͱ��G9������t����>�5�S3D�=����~��w���h@�1>ߒ�O�yD1�J�m�}r_����%�P��`{�[�jC�YZ�E��:,�b:�#	�(Q�
E�-)J��d��������4}&s����H��Ry�C�97������"LL+��A���͕�N吡�7��w��%4'g PVFb�6+�#=��.=�s�Ѯ2���$�-�`�5fo�N��B1��ic(�O�#ix���-��w3՜߆/UU��a4��x�fƊ�	X	N���{=�5b�����=N�H��>o��y���<��'�@������fA�8@"b�FqD��(�D���8��Ր ��YH�F��`ӽ%d�}$"w��G�iXD��I��x��1�}���6�c�VjZbHdŇ�H�N�Ӹ���t�&M�@^��S���X5(:��Lص�ci3ê�NQ�6�j�,���H��4 $��Y2�e�[%���Mѥl�j	A�zu�**����a��/v6��Yn�fv�8q�_G��ӀY������P�[C5��l�A*���Ԓ� �o͸���[ֵ�mj�e�r�~s����!"%t�
G���� �D����>�|�Ʋm�i���xG���c�F`F�l�&���:C�{t��uu��}e��,���8�M���J���n���϶��e�P�����@f��Y�->����v������갶�}&iR|�v�>�����k��/wX�>�(����@[j�o�+}�q�	-����z"�qnݢ	�Բ�G�#�SI&��r���	������y/F��t�B��x�ƔB�Q#��#HѤ��܊��$}x�w�L���CY�����":��u���|��{[6f˘莤]�n���DA��s=Cǖ��%0dr��9p=|����~��1���ڧR(���y�QE;m!˗.W	�kh��d
�m|}�����x�nж��#;M����){[N�B�ęb��d�E\��L7��[�@�08�%��I"" ��`#�i�C,I����+!Tc�B�y`�9�~��Ş\䃚�<�߼vP��ͤ���_���%��&
���!$��6F�pq���� RC,Iْ�>\6;J��opw�q����=i�;�)�ɫ00E#��� � ��F9�G�1��ۂ�,�u�&U����I"%��EAUWjU´��J �Æ�g��Z&�><'�9�ԇ��>����@+�h	��Y�H�4a�X��OϾ�9�`D�tr7����9����SCVP��ˀH���F�\�=r�޿�͞D� ��n���4�&%7*T�e>�1���D\h���C��I�ŉ�7
QMS鍾�	�WO����7.u��vغ	��SR���:R��������� �O2r�Kh`��X	UN���>@a Ɋo����K���.N^�nӇ� u#���8w��Hr�=o=��宏�#0����p�[l����9.���%ZDU$�e�O�#{U��бڍ��N������:�3������ Ű�N
[�YtT	
R�,�DJD��0�&�d����*t��<�w�Iܸ1��˂�W�+	��X*"�\.���r]��6�/�ˈ�Z�-@�e�y"R�yn�]�II��j�1��l>���� ;m0ׯ��3��G��S�:-�(�a_nZ`��)JQ~��z� .DJ""� ���+!F�J(0t�("#�����~V�O��ohr0�<�Q�"�Lssڙ0��QE4P��Y�pOߊ<sL�гy��u7���
�>��8�A_��y3F*���PW[��Q�	&��R��N�ms���I�a�%��B�HZ����� ��=��di����d��~;����4�����|b|N@,�4@~`@�d�����kR�@�9�)A�k`D�a�z�7Θ�7�F�\t�Ўѿ��%�B���������n���?'i���!�=,����T�rHn�VxD����f��2������e�E���	U%34��Puu��Luހo�i�����z��eH�]2���^��|��'%xG����"Ni�/"G�3��][�U���A�����b�!��sѧB�^�\)5�P��u[X��Z��
�I>!+.����47��F[p������m~�d��d@�"��^a��@�2���9���ݝ� ��'P�(��H)EQ\���a�̼��u�z�o��g� (�!R�VE��Y9M��e��,C��p��]�����hvX ��?�#ջ��A�	�=�� ��W�̋�X`3�`�7�.�OQ�v8MU"g�}H�>U��{��xߪ@$X�W�~�}揎@$uk�^�.w�*��c>�x2�&����ƀ��]�E�<�p�����|������p��)�Ѓ���O?çyb;
&��1�v��� -\/ QD���⿻I�Y�:^����X9	�z��ť(���;~W����pM�UUUV��Q�Z�P��1���3�I�0?L?�$�8���2ÿ%�
�idC��A�$�CI _o��:MnE����M�cRB���	���[���3��&jv;��<U3�쵗>j����"�H�*�'��~y"�)#�H(AX��Ă�!�>�׏9�{I�I^��9,�4!(p	�����>_��O�1�_s����ry�N��A�����;�Cix�iM�s��� J$�_��52����Z�5�y{G���le�uJ{po���d��z��Y.�a��z��Y�-��q��0!)r�=I+���G@⧔A,���}��9/jP@x�>�{����O~���u��W�׭;�u���y��P�n:	S"�*��I�LJ���_��>}�P��c��6��F��9I�=�C���#��2x&8�i�(�|b�hcP����f��G���d��s5;��9�w8�L�#�����	�z���������\i&����(VH�_/i�$�C�~����js��n&��������S�+��Ŏ	9��Fmz��M#�H �H$�� .U �^�g���4�ߪ�y�aĔ��F�����|�Aa(�6<9
	�
c��쾨."UP�JWygV�ǆ�y�Y���E����\�?���c��rp<І�%՛2=P�8�,�qB@r:<��p�Gn�A�����lć3�2_�Wz~U|K�����	0Y�	��QG=͹h'ع�d�94�؍]i4�2Q�$����,�E���Pu&S{�i{칼z�Gf�1�G�7�Dq�� @�rπ+4�Ok쁀qb!���H�����(��gq�����@tN�6u�>;p��h@���+��s}UùW�M�_�~�+uXG��e`����6���Hh�r��]0��p���'M��L��)����uf�S���W�3U#$7��g�Fv����N�t�3��fh�����j��[N񴯽�e��#8�*��.C�[n ���ISŹ3�H;�H�x�p�������]'��f��N��$�9\����BϞ�x�W��� t#]�+ʨ�N���R!{��h���)�i:���r���)�2�b�T�^S���3�F1��Z��W�<h��������ʨ4���O;�;_��{l��?��o�qt#���PU�x��$��>����7�Q�*�ó����/O���ћ�W-���o��ew+��������DF���g��o��}:�Q�$߇��d�ן�S>
��z�t�/a�Ֆ�$n# =e-�q4�
���pf����A��*ٞdѽ4ԮXK�(�ϵ��I7��Z�"��)�G�Xa�;�<m��͒�4c=�U�c}�;ϱ� �Z=*�[��� �aM��^���b�Z
��S���*$�GHɳ:bN�o0�>���j��*E�5�Ӻ�\���/p�)�.e�<��1m�-�e�,Q�Tf׆Q:�� k��?�
��6�&b�fcN�z����o�Ww7�ٲ�Ca���[D�9P���<�-���-�\=��DR�gb�׃� * _|�������rta�&գ�7���F��H��\�a��z�����7�x3c�$���p��Fx�\��4=�j�B�'��6��q�6�@�ؽ��?������U�Z'��z`	�5X�gm�4;H�b%�@�������	�Pk.���⯎��J��珅��I��H/������ٷ\�:����8�ɝ|E,E��[�u4�J���PV�ބ�-�l]��`�q�����)�g����Y�_x5���)/=aϻDZ6Yk���t/W֡��A��ϐP�Pﮍ�D,uP��-S��4.���ж�#%q6Cj��迎!tB�^-{����v��8��37,�VpQAޡ^�hY}/K�� ��&�T�g���K�Ov��x���bњx��?u=�����^���DȁT&�d/����?.��-�%"	$A����RE��� X# F@D?�B`�"��Nm"D����ab�VHV����	�6��=��A��BDa��Gl����FѢ/�`#i���8�*9��݈�Q,�:qS�X�����\f�X���E��sY�� �r��=9R]�	ٖ�JKx%MܙG������@�
(���S++������
�w��Y[�"�㮂���H�__VJ}�d�8-wŰ9���@�7 �0@�ܻ���� ���ǣ�:\��s_%���~�z�� �uD�'�>c�0<�)Jv@6�H�B@ څ��X\lv��-G���dNO�Of��G�«GF��w�9�U�;��(�E�77 бC�s(a�AB�J� ��q�>���L�:�弔|wf�[�����/k�Kw�M�>��ɶa\Y�Kkt��te�JH��x u�6<:�e�	�MGn�V�qu颓͒�R̆:�0R��a�Ϭ�#�6	�T �T�QW	p�G���V�Bē}��o�+뮋V�=��BnX�^�<Fᬹb�k[���Sӯ7����ڶB�X��7���j!d������-� �*\�n� k�x��05"�x_|L��`v��J�v��c�3]�h%$�,!�0s��ޖVV�\]���<�6�P"ŒTT�`l`p,\�*05���5% Y���J�ba����a��7XM.����c��L
I4Xj$�5�QC	��٣a��
B��P�t1��$�d\Y��:�H�[��5��P���L l`�z������1��0N��9���F�P�8i��-������?��O������<�����Ѧ�	�4�.�'����Xd�(���]�s��Ǫ����ۉ�{UYUʓHL]0.c�/mLΞUy=mt����2zf�^�n�gk���<�ŷ��<6�I�<�mqj��������"�b�}���(��v���a.e�\U"����t43S%��y��&w74}8;��<5�z�;�I39OACX�Ɛ�K������q:�v�,�gwm�ܶ��B�U~�������A�Z���<eDdADX �X��(�A��Bl�#,�d3����65'wa�'���?H��lv2���=2�7֒B Q���I ���oW�Iw�v�V0�� ���wi��W�����%T�&������Ry���Y�e�s*�*"�2x���P�N�Q�,�r�RZ�D�>M��~/��u"�����d���龋�v�UZӍn͢b�����a̶V�W���h�V��*a��.��gW���$�*ޯ������!�O���DB��ǟt�rk��o�'G��ݎŋj�	� =�X����X� gD�c�o�Q	y���4��[d�Dsz
UP�L"��)f�G�����9�ߟ���/��`@����0 `rA��U�g��s9������X��u��	{Y��ۚl/�R�[~��p�mf��#W3g��Lւt9k��&i������ȓ�/B
��Z�9$2�i!Db����m0-C��	T:b	�����.��ܟ �V�9aR$"�aj�11�0sJIq(>	��UUUUT�V�,Ԧ	 �K���GG�������$	� Xտ��%m������o����������ۘ1x�z�caDR\��]��c��*�6.HJ�_Uo�9A4?'$�хy�I��UG���d(��Z"����^� 3z�	�L�(䢏
UT� ��b_�#���ˈON��i��L�^qx~w�!��0�Lk�����[8Ŋ
C��� $��<3��A�tm�z�uXFc';;:� �&����S����s�ĸ����A%)��3��3���n���0��7$�\
��O�&o#��w�ӥD�/d�Q3K�Ÿ�=�И�3�e���V��8_ ��/�|a�q�=����/#��q9M%T��l�ru�yf�,���1��O��QBk"���-�5�R��0͡��� <P����ʈ1�L� �@(�8=Mtd����K|��<�2u�I��l�����y5!�Jt�<k$N���C��T�c@�H
ӡ���
��e(���*K9��J�ngU	xd@��0�@ҍF;�:���e-#)@j�-@<��L�4�[x0n�Q��1FM1|i�^,s��k�l�y�'�	���]_N3�i�e�{�ey�F�s�{a�J���u�*���ʝ�v[Y9
5h�s;+�����u�	$ �)P�_wk��y��^[wZW/�^�K��i�	��="���zt�+tK�54�ܼюj��5/�m;��>6x	q�u�.t�
ڮ�B'�6����%]w^Fw8�J2 ��!P�'�8�,R ��s�? \�рD�`��ދ0�x6��oqPJ��2Q.��Hf��s9Wv@��_8�M�Z9�D���p���,A�q\w		y���x��=�s��]���>/�{A��w�������q(�/��=>?�*4�Z�=K��P�����˜���q�u����ήY>O����/B�+3yY��y� K�Lk�V����0d�=I�eu�(T�">�c6�p���U���O���>ӧI����l�߾>�U^G����WX����KMc�5V^��{�xm�X�H���o��{��]���j(?��Nd6i���C���Yȕ+�T'��~�#a���Q���2� ��Um-eW^\8�{���9?���_�e��f9ף~���sg1��q-M'��R�����差���깏վ@[&�Vl������|tZF$K.\�r�5ֻ7z�,��ucǕ���+���ZXI�P�;�8�Q�-:�l?O��S_��<�-&���Z��_J��O�|��������774IU+���s۔��N{�m#)��W&�3 �qAq@�!�Z�R����_*��
��8u����f`��v/ۼ�u��ܢ+܋�W����/sR��C��?b�Eم�r�]���ַ��2��L���+���e��],�[a4������1T(w�Æ��R��c�5{7�����1t/#)��dPs��5��]3� R����X~�#!N����z)B/<qcj���H4�Z�fhW�X���e�V�VM�ƅ�fgd����
��=���֡����F���U���A�a���
\t�Y
�.�2#VU��*T���:�5�X�M���VW���w��Mt��I;�1����[���޵iZ/]����"@3�F��uC��l�q�9ԃX2Z"�ro@Ly3I;ӫ!b�)t�?����Н	2�1I`�@5�Q^�:"�R�(����(�$��Gv�Lb��\]/+jmL6��*�
�yra3��#��I��N0�ٞu_u��J�7Pg��8�7���%ed����=9i�[���t=��J%Jx~�퉇��OşGӝ R�h��-yg�I�	�P�xG�kgf^͚;oՇ߰�s?bwNi�O،R@�ȃ'�Te�&�(�!�=P/~�UVԱ��������?4�X��.d���Q1��y��h��wL��9�.;d�@s�y���%�{I�1��rpJg�^,�0'6��я�́�ǋ�����a:o�b��6L"�U)ja@�5��F��*<5ﶂ�0@ф4�ym<,���o]��?������x����?���W�6�Yt�ϧ�wu��'zy�!�b[O��n�]��1f��+m����@�˪�C��I�?����L��lK;ȩ�#�C��9ǳ��}�? }�'_�����ߜyfj(w*>�q�ؼ\���M�2̑��sx{�Os�_p*�A�O	@���}\N�`Qz+�YE���3���|����!�.�Ȓ�#����H)�E��-�~�����Ҋ�o�m�{9 ^t���P�k�ϑ�N�/��*�C�|z���ם���[��DL�4(#���Eԁzˌެ׎^��pny���3���]�������!:yG�`̀I���V"�Ĉ�γ��Wf.~��>�(@&2$E�$�0�C��H�����_�/`:�3��̓�w9O??F� �QZ�Y��B�,�]�y�7�\��܁Q�qO�����@�����P?��f+щ�ȎTBIxF8�\�|�6��џU����!u�AxRj܁�97�<��KPX�dDm�m�H��[F9����h[
@�4v����' i4��mI$�8���*�#Ku�S���Z*�����;8<s͍�M����e�V���|%yZ�1�39�]�ʺ ��)���S���g\v�.��r{ZΉ�	B ���9�Yx
I ��}��7�a��Nɑ�r3z�JQ��Bӥ1^#�/����f�^k���P���+��`� �x�Jn :�]v�㸃�����������`�m���'|�/|BFML��rB��֔�@w�5�T�",�������#&ajh��]Ġ2��ɦ&�CHi��s�Ctf�dWs[h�A慜�i�� ܶr�=]*ln
�̦�9��yF�^48�.LK+B<!���A@ m�ǡ��
#�(�N�8	5�I!<�9�dg*h	��E5N�i]�ĢH�U���)�E���0�r;��O!��,*��.f�,�;����)����t�&J�V�/�e�`V�K�_U"��Xȍ�B�;�q��A�h ��ڂ�b�?�IM�����6u�|ԍ@x�d�l=�͍�w�6��]�Vp����я.O5���Y����Ҫؾ̤���ߌ���jBA4st��D;�ws��W΋���wC�of�ӇF[��Y��*��zin� �V1ګ�����r��MV��%�g<U5����ka��vj��BBB��yq%�a�9 =����nl��
��� @70��@☙{,$�r���"�L�
��ĊXZz4r�I$�t8:MG5���H���v��C�N��	"�u�q9`:�� QrX�A�p!YNts�9yA�n��/6�P�����BI��#�F*�N��0B+{��I���Τ�A�!��t+����4�[m_pT�˂����T��������91St뀹�z����7n��3�u�cr�v���+4��Y�D�HS��6ࢃ��s��ez҂r���pu����`�c3��9����{a��`�i Co|XgHR&"'����Q��b��8bt�WY�S/'�!+o�.�iz��u���s���$�E`d�#�2jZAb])���e�����Р[�����w�I��ʒ�4"�O1����BD`���oP7���9yiO�\����¡`�	���6A�V`=a��7����԰-���7�s��(����u��� i'cH!��ӜL���xv�l�L
T�=�de��~ �l�W�P��̄FF$�t qh��Is�/ �0S��`dZ�1/���;�riD��]�Ĵs�"˩D<J\�K~M&)�H88B �ƴҕM)왨=ڭkϧ�.�G�.���]�B,�������۸��o��y<���S1],T�S�3 w8�%2h,%������nr	a*X��p@Ҭݧ-��CIM�7ĳP��7���Z�	b� o�;�[�"�����s�?V|ĵ�D�L�cj������Vhᑴη���C�1���L�IU��J'4^�T5��C��Dj?��W܃qxMSXi��6�bfh��ʠ��#c�m�{p<]5C��yJ[	(lɻ�,&�C��C��g�쌆Ƽ�Zh�dG(S�7����Tw����7\�\�m��8��
C��y����0r���K�`��:]�Tu�$H��L&w��2N��2�\A�6 ����a(�*3
6�LOk�XLԦ�,�!pL �# E#$�������zB�<�Y�c���P�c�2"1�X�4�0��M�d7�(�A�&�H0ޠi6�S�l`�.�.�d��aS5p��1�6M�y���w!ܒJ@��Tg �Sk�5to^0oA.}��.bǴV��͞�΁��3�e��6O�Njd@��`s0!}zX��V鱭fN�7/1�>�Q���: �Њ,�EPI��+̇��3���[�w�HV#ܻ�P��n&[���"�EЁ� ��X:�7$���y}��:�T�{SI3"�á�d-D�!T3}:�t�k�:jM����g��B��V/W)�oۀs��m��<�sd�+��v?#q�AE#�n�]�qOeD0ݝ�� QtI�c�L�䇖l��ol?}�r�� ôo�X^'	fa�� �mGc<N:L�������Ŋ,����y���a��rD���<P�NT�8�WF�=k�o(�!
O1�q8A�pE��TX��đ�dP�+����0+Ͳ�ϣ#�V��HC�T�U���;A�Gu݊� ��F�n�V6��Q���%�`]؉�w�t�A����׫]�^�D���UZ�z HH�$�L���T���wgi�yP02��dC�Mh9�N� ����)���CDC�9"؀ �6>'2�n��}�y�ȧ �Q�-���?6���$<c�N#��!ŀ�a/�(Sk�<2�Hi�l�a�/�e�U3ذv���
:����a��z0ݰ��� v���h��R*mF	 ��d��)���gv���!�]!�k@s����z!��w!Px>`{d$d�Ο�Q��V�-�Wי�kM����h�`�Gl)U�(`LP@�8����3s�����G�������'�p����P~*�^�4}�_+�=��lyd��p����B<�*2(��9��Β(���ߧ�Z����<!������s�~�*
	O�G�
:�0��[|�>g4�$=�����U6���u��g�	'@ }9�.t!�Q�lR�0t���ێ�QX��tC��o"D�X
fd��8S�e�5 X	�!Ĉ�3Lqۃi�u`������pU�ML�#F �`�M����Iˈxy���.�$eHX4g�n7;�./DY�EV*Ǒq0A^��1��P,cNf�9�2Q�ސ��pH"(���5�jI2u���wYQUQS�m!�͎D9ɝ�ې�DJ9BFhI �6ˌp�:�9� ;��a����H�lI�~M%N�mң�<Gi�m��z����(�pdH��l�@�	��.���1m�6�-���S��L�g8pSSH(�A��aFK��N�&�����fx�_~G�u��r=�PTJ�J����>�����'����;ݏ�����\��>�?�����_�p v�]Hq?�	
YzV��!�h��d�t��A�u���7f�N�u0:��Yauuh����])��[�C��HhQ$���8:0���M���\'�~��,$:~����N9�o��0:��8��ì(���1y�a�G�AD��G��lH���3�T��M�o�ZfA�P�{	��)��gP��0D��'a��=���r:H�
 ��{ɠt޸h�(YmlaD�N��,�'�C��~ĖN�r<<#Qvd�;�j.HI'K���ͮx�0�= kٻ�5 �DR�\�%�@�Y���cav�l��	��t����g�C�+w;�y���;]�͐aS��Ѱ�C��p6���P�CEܶ`���"C��IAd&���N�@WR�G8BI��YhخR	Ẫ�g>�N���'��"(�)�:s5h��-!�qF1�� B%�bB��k��ښV6`�l(��#�'o���&�e˓X�6��R�
���[��#�(A�N u��O���U�4l���e����]�n.l⪣TE1#�l�z�^��h�E�sE\���V�K�`�DY�	�PCG{���`���4 )a�"�p��	���z�6�wH�:�Hś�Hzm<���@��d���2<Ί�� �_D��;���@H@{N�d��%ɀK6�aVЁ�d4������4��2��	���壼�6.o�?�0�Q����"�db���x�II�jR�L�D�Ĵb�,Ctc Q��`����7A�G;�Ȑ���KS$>���P���X�"h�� �E�g�,�#b2 E"/�T���H����@�h]\`)/i�0@�{�F�D ���`�QTZ�F�݄m�
5��r��cW"�!�+'z��O��Z�*��Σ�l�'���GzX���|iS��	ҌQ��e�E�lx%�x;]c�r$��F�ԥRH���0�;�:�*���d1!��°Z�3rݙ���$@*���`{j�-}Lتo� n3:�ND��ɢ��`s�\`Ă��|a���q""#����J����|(9S5κά���s�ǃ �d�Uج�ffh�����S/J6��:r����B1��˜���z�b�����v��^e��U`������B���I��K&$��[�ٚT<��|�)��$�SԒ^/Q���[������`	"4¤�4&R�@lwt�v�H�P�}�S	�$�����6��Fr8�*,��`?u �%��chc4R�G�TDe�DA�>�.2�dȇ�m�<��!?;< (�1�z>9tCf���^�(� p�5p��� 1�Q��<ŏ�k��P��<�'m�h1�"M����IW������!��Y5���.�H.}����Z^Gg#�Fx��Ƀ��(��Wy�"B	s\�o���&��{f�{4B !2� ��Ǣ�z�%�~�ܾ��V�JD�(�
��k�}G��<��"�"2*(٭@�1��xH��-Qy�aO v�;�T����I�Ñ�HR�n��ͮ���?QJ&Oda@s\�bI3���"���5R&&�gK��%�D������E,��̥Q��P��b�%".��=�F�Q�'�=8y^s�<0^����D�cw��(q�'2��$�;J�	�Kb�u��[��<������k�'5�i�E
�5BE�!��ɂp��c�\���iJ��L�*]�~�;P?RT�B��sLC��f��!9���1�����/�����P�:����bA�W��x�k�`�f�]�$qE�\o��� "�GZ��n�,�L�p4�V LCm{0��`m�����lgU��W:�.����ʾ�s���Q���� ,�����(iC���rC�d���P�����@�̀�Z��ut<	!Hd���o�`��8��v�g�����ϗ�.�0�`P�!q�Au5�V��4��i7v�JPw�&��2��i��� K����z'C�������`�X��a:Ѵ q��I�
��5iq^㱋���N�#�ZD1�XW�dn�:
^V5$\����&����,9�c6I6��jfL⪪����K��H)�n��tQb�"(�TLo�0@2"(�S8�s��}�z;��k���p6�s8G4!T�:S���v3c��C�=C�P*�Qb"6D�;$�&�1�F��-�7d�8>
���z6z͍!��q�8D2HȌ���|�(�$��!;4�� �\sg0����ek#��{�y�K[�;�A�3w��&ɼ��n3������Tm�΂���켫'm�r��)�� |�i����e����e������3����kX@� c���jjsu[�`o�N|����;�]��;j��q���XXbɊ�lp6�r�t�9���HAa�R��EAS���	4>ǂz#!?EC�������$�r���7k�(�f�􂛂(��:I[u�2�տ]7I����my	c4�u*3*�p����B� �a �	�+�/���
�n���R���HH�*F�QP�`S�e;�/�I>�g^șǞ��80b@y�1'ɬSr;䏌H��X�X��� d����יI�/&�b�Dw(�$���6?��2iN&_��;.
YD�����
���C�X�����5!��tw��e�Y��a�Vխ�`;�ϓN��� �%�g2a��La�$f���ّV��4mSey�!������.�
 , ���߻Ԇ�\YJq�X��HHw��]���y�I"c��,'���=(a �Ar�zF��Q�s7�E���80�(,Ќ`$TK�A�QETA(����Y?t]�Q�yy�����
�	(���&�a� 0�yynR2���S�<���4<�||J���=���X��TYC�@�Ct`Pr0*pkdq���Ad2.(D��$A��Z���aiF4|���ZE��K��fBBI=x���P�У�biS���׸�N[(��o"J�9�<���ԴJV���>���ntp6.Pϥw{M������{���Q��RB@�����������T`G��[�#q$d�El9���H��Q�׉V�`e4�����0����V¡�\��ά���Q� H�$ �!�g���|ɏ��X7�B���1U��\'�2S�Ou��oXfn�>��M!�wE���'e�:���d�i�DEO0�F����R��F�1��1a��Ɠ%����H�F�_�͠&DHE`$��j�ab k�jC�	�&�6�G "i%�Q�rc�~�V��N������Ԡ�j
D�P!b�:��x&=jmbX���D,��0u;Ġx�m��Q�K=YT�4�V">{�RqH����.	)� 6��I6<g��΍P��C��p��h�3�p�8�>����a会A88>�`�t���ߖfF���n���H���`�
0⪥p�@�v�B�
�S�Χ�:� ���׼x��ƞZUkKim-Z���������UM�V�2ކ+W��'�r���S:�a@� �D�^�M$Q+�~��B��Rd��T��yY`%Dq $D1$'���a_qb�?�o0;ɒe�C�/0�¤�\Q���rk�M�G���}�gG�,H��w8�?��tA�����=$&`�$�_����؅�H�2"�xD��,��@Ad
c#�-���h���)d��g�C{��dg`���m���)�H]��jjE��M+`b���&ƶ�����)�=~�ޡ�M%>$
d���w�{���6�yA�y�(dA���(��b� �2� �2��a�J2�kp	b��FR`�X���C �b�
!&�8��e���66{�қ���R N|�
i��wޜ��P�ެ`���|w��D��E@�A�@VDvۧ0o����m95
pЉ�:�8{o(h3g�V�ЭV�f��D����D�:.��u����#���V�w>{�M��vI�N1X��:����`�翕-�	Ra�������2�B����HQR�,�`�P>_}���"?�`�35T F�����j�8�_7֯&�������oPP�x�0v�iy�iQc��&��A�%��%��a�	a� VhJ=�}V"n�@A�
 �"�}�G�ڥ������ {�����ms���o�b�y� �Y�,��0��l21��ҕ�@2)٠�`��L�(�Y(�J�f��+��"-R�f� Z��X@a�"��\Źҵ-����&�$�1@Bd�B�N���i�*	��q�kt�Տ1�D� ���t�Cˇ�@@��`/J ��b$hP�6�);F���b��m�ɻ��=���$�Q���~/f��2_�:9v��y͗1=����T��v�V���*�42m�L�*�*"[��d$�֠u+�x�p7���u�aچ��"hV��/�U��H�/ȝ.����������Q6�H��FX0U�w��Ј��D��z9B� �-�͠ܯ�����u���uN��J�l"!�_��ݞ�==?M����hE�fǖ�%,�i�дFKӷ��:;=5��' G�m�m�T��qP�%��s.��b�sv$�hh8�zr��f��n�LhUL�~$�!%<�,E��� {$߁�{���Y��*g��T2�8� Na=΅ �%<C�}>o1��ll|���.�o���%<e�vٶJ��5H�!12�H��S1��e�D���L�(� h�"3H��E�HO)�)ZQN#
���<��2�$�ۖ6���z��l�F��ET�$ c���ʯDLi���(���1��u��I�e1k2����~`���ag�����&�s#UH�A-���d�R>��80hM�_d�{���d���BA�eB����~��]�M�M�Ur	"��W�!*Ğ���Г%��(�B�wK�Ѹ�f�nji��Q���<lۋ�"U7����Q�u4)F��3@��4I�{W���F�fB�2�́}��b� >m�"�D7��0I
�.�Su\�S0�4A���E�I	�Pi�f ����
����(��50����H����.�+�u$���Bu�l��I)�(�5��j���0�� B�H,D!Uщ�їɊ �ٸ���6�T�gL�=���a!�x�I11X������(=h�@�"�L�eH��:زSYi �+	P�	D��$��(�N�R�AQ��$�Ć�&��X i #�0�)&,B��*�ޮ�Oxj�AC?��Ld;ԑ$HG�i �տ���5�֎�@s�}��M�d� ���o&@H:m�{���H��=<�7 ��w����4�� �KITQ	�@��<|�!t�Q���\�wR��^ϸg9�u���5�{V=Gç�T�m\Ң�(�E.�!�I�3h���H�O԰�����(^{�Q����(�3A<U�%�`�Ix��
�B(kx�2�a縱�t�vR��(���&R�G5�l��v{�R	�;����,��D�n�ۆ�V%C��3�~`�7���`�[ohe�`�=DF����� �>m��oX��L�H���uQ$�������A�X��_n���ev*�Ju{�#"z�I	<v�:�B8"=uJ-��'HW��û�v��u�p��Y�g��(8$��h�j�ŏZѼC�A EbňC}���1�:�@ی
,��[
��� �2:����j8�"�&(d��B(�-H�v�IpЎs�f�ew�0��P`�7�3*e�D�[���'26ha�`����,9��*�K��α�Ud��8����/:tv�QD� e{�*����3\�2EC�
�k�4��"��BqJ���L8�B0T+0�����)R�H)�,2������#	o:m��`�d$S�4\�&����#��v���9����7eq��xE5E�E[	"d!x�|h v.����4Id)�� ��,���繢��邈u��R��+�:��&_�;�҃C�3>�/�'�7�9<����! ��@�<,�6���~��k�} s��,NQ�=<�Az`�E��JfBP�����{�$�Kj�[���Sx�f�-�Y�!Q�$-�CCc �EH�8�9�
tう%�'>a^8�.�pѮ́ �X;���J�,k`�[Xb��ŅKl��,��L6�E�Q��e!e&�K�Z�g#	WNi#\ c�wB�ߍ�WK����m�B�h�61��&96�l�|*y��=���2 5ն�A�F	2`{���3X`6�K����s+��Eq$:�P���1v���PϳP�
	R!DaRU��|>�:0�a�;W�3�AӅ�|s��F��qAbЅ%;Ȅ�BA�6��	�kR���)�����̙<w�,�1뚇�Y�;ɛi�	�$�zx��1p��(��KB�2�OT�� ��W���B�Ah g����dW�	7�!�)C@N�����Z�e��z�y�	�T�x�A=�-��-F�@����>���1N�G�0A�pA܈������?%�ܒH��f�� L`�b�4 xi;Km@�k*rwt�~q~�Ƅs���@�b�" u�|и~�9�LX��DhLs�2,H������$"�(���2# ��WF��"��	�k)L�  � 0�	x�E�{[�̟���@��z��|�6�f��7B���Z"{3��L�8V��d�jc��A&�խ.��� �T4�u&㡂K;r�A�8X��ɬ��g��f� �:Q�ٻhr�m�j�zH�I	$r.!��_O��sl:Զh4w�J�a���mok;�0��0	�Tq'��f.1���Uai��,@e��WGo�1��)��dĪ��
��8��A/i2K7�L��= `:Ǟ�sǂB�0�YC[O�c.C�]�8��g*�M�O����k k0�KI`U��I�3�w�Q�FW�eDׯ΀ȩ
�;�@C�`�D�@S,�Aa$��γ7��N�>y���'U	�#T0CLtJ̤��w1����|��@�$L��f���<��9��ڤ*~G��F��.�L^h�N��C|	|BŊFy�u�ѽC���$�,9�^i���sÀ(=@�GY$k)�y,��.#e�r�hI;{n�x=�^��3�&-���N�0�t~�PH��� u��M&���RjmhB,R��������[E˺[�}��k��d�Kto�!,X|dń!��6����E�f�c�,�Od٪i���ժ%8V�H��➓��'�.d�2湙�R34�O;����,39rZ-���ZB����A* �ʊT��$��r�=��r,!�pZ��&�&k@{���X�{��htm.[�@�|B����8�a����G�/��S� lbv����~L�Ͱ�����` :�6%��xP��Dl������$�X�Pb@)���������EK�
����6�����lbJ!m�=��������̞�;��� �qOT$���pv�Vc�g,�����޴��ܵ܆���֩��>�s:�<N4��Ў\6��qcZ≮����$$�F��l��ͦ��m�mǔ�	iى���a���y���"m5��)�a�C�5��q8���aCfꓢJ�q�@e����Hͬ1�
� ���C8��A{--�YUx�8�d#��T�ma�ȁP�4?=����k$h P#ƃ(jn���a�nѲf��TH� �2�B�Ņ2K*�KR$.fW�ˀ��������81��(Th�].3l �t��(%
�T��#4�<f���@�4��5�q��!F��6M�-�I�T\��d]�]1��Ҍd���̝5�dG���$�ɢʣ+��\��H!MU�q�ƌ�DA`FP�A@e�6�`�l12J��DhB ��Ĉ��� %[Ich�cxo��͠��~��8�* 6�10l�iLj�H�g.���f��I����m{��X�6M|C��5"n����BLC����5D"�9�9v��Z\�ʍCg�S@[[�
\�Ҋb�S2݆���@��Q�J�
`��e�B�ځqQ`BЦ���\�l�6�*A �,"���l`H���`�K����
(f��� DG������%8�!/E�:�d�� Ê�	 �DHE��FB�����x,��K, �%�>OwC�ʔ��
{��-Oi�
T��6w5�8;��K�:Yqq��R���)��Z���!�h�I:�W& �G�]�{�@�m���S�I�Ga��6e�f*���������#�� ��J�l��b�x,��Pt��j�����+Z��3��:��hTC��l8�q� ���~�e-�#�	#�k!��NQ ��B ���ǣ��b��kM�i������%$�J�|��`/5��y���v��'`
���j]'�Va bi�[=�/i�8iq�*��v=����_�FK���a��ˇg�3�s9��%U3Am�Cr��AU�#��0a�d����4�2rg��'�F^M��XeR�  ��뱯���!+�]��UMF����50���ók_�Z�g2��D�t\��>'3�~<)�9�xFư~'|XV�� ;��kCx� A��H7��<7��� �:�_0�A���R�Z�j�M�7yD�ʮ}hA#� ���@6	�p�����iÝ}����̠c�ЦՉ@`LP�B��KT+)+bȋ��1�&5��+>##���E%	A!Q,ʈ=�����G�ꙈvY$h��U�4����;�
jh>t�U�+ �"LR�̏i�����.r�)	 ��),P�HBt���)� ��9^�C���;�H����;<`Rb+1 Db � )  )	�jh5!Ӕ� �)@&��0������>T�M$Z��'+���MrUԈd��^<�Rp��d]��J>�qsT���l�"�"�����S����"B'����J��q5�.�m��I��n�D�$?z�����@�iȃiM[o����B'��~�.��c�?�\� �7hc5�~���4��p���k*d?�A�$��;��u�3�S���}�a v�� T����d���5�$�=W��|?��!4��(z2�z*�Sd�vA`(,��$���� �Q"�k["�SB�b	��?(���R�\�� �f�b��*DcE	)X���eD��*�"E����a�FKJ����03�Y>�A-��g��}�]�6�����XHD#	1�}���x��ߓ��8��ܸ�]�T�l9���*�M�.M�f�1���,��}�����\#����ҏ��(odhZ}�{�)��R��$O?�D����)�]��P
