# basic crypto so the CS students next door don't hijack my drones again
# AES-GCM because it sounded cool on stack overflow
# TODO: rotate key before the competition

import os
import json
import base64
import time
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
from cryptography.hazmat.backends import default_backend

class AdvancedSwarmSecurity:

    def __init__(self, secret_key, salt=b'swayam_salt'):
        kdf = PBKDF2HMAC(algorithm=hashes.SHA256(), length=32, salt=salt, iterations=100000, backend=default_backend())
        self.key = kdf.derive(secret_key.encode('utf-8'))
        self.aesgcm = AESGCM(self.key)
        self.seen_nonces = set()

    def encrypt_command(self, cmd_dict):
        cmd_dict['ts'] = time.time()
        nonce = os.urandom(12)
        data = json.dumps(cmd_dict).encode('utf-8')
        ciphertext = self.aesgcm.encrypt(nonce, data, None)
        package = nonce + ciphertext
        return base64.b64encode(package).decode('utf-8')

    def decrypt_command(self, encrypted_str, max_age=5.0):
        try:
            package = base64.b64decode(encrypted_str)
            nonce = package[:12]
            ciphertext = package[12:]
            data = self.aesgcm.decrypt(nonce, ciphertext, None)
            cmd_dict = json.loads(data.decode('utf-8'))
            if time.time() - cmd_dict.get('ts', 0) > max_age:
                print('[SECURITY] Command expired.')
                return None
            return cmd_dict
        except Exception as e:
            print(f'[SECURITY] Decryption failed: {e}')
            return None
if __name__ == '__main__':
    sec = AdvancedSwarmSecurity('advanced_swayam_key_2026')
    cmd = {'type': 'COMMAND', 'cmd': 'TAKEOFF', 'target': 'ALPHA'}
    encrypted = sec.encrypt_command(cmd)
    print(f'Encrypted Package: {encrypted}')
    decrypted = sec.decrypt_command(encrypted)
    print(f'Decrypted: {decrypted}')
