        �  �      ����������S9��g�Ҧ�&�I�k��            x��SK��0��WL��uЋٔn����]�U�'��%i�&��#k]L���T'�����wo�}��J�ew���B���u7���|���z+I;+[��m1�&�)~q��lV�w���3�/��+�C���>�2�r�=�ǆ�.O�I��#�h܅�����3?�P�E+��w��V;#5��&�t�t��"����Ӱ�����o� ����'�ǫB��|���`��G���\x���ym	��+y9�_��y�le�"��Q�8�,���p=u=��S��#m=Ϯ�֩�}�q�0j��`6Zm���<g*����7��̓v�lo"��+�Ι����$^5KNX�T��k�&<��[X���Z�H���<*Y��*b��#�%�tL� ����F;�U��]���M7Պ��x��o!̦kx��$i2�4��7:D	1Jr�C�%H    �     [         �    ���������T�2äGd5��%�:               �   �   O
if ENABLE_VMX_SUPPORT:
    SKIP("Block-detach not supported for VMX domains")
    "     Z        "   �������1�N���4�e�/!               �  !   Nif ENABLE_HVM_SUPPORT:
    SKIP("Block-detach not supported for HVM domains")
    |     -        &�   �����(����?r���m���{              �  �   !    domain.start(noConsole=True)
    �     W        /�   ����`9ɲY?kYx�Ea"_+�l��              *  t   Kstatus, output = traceCommand("xm block-detach %s xvda1" % domain.getId())
