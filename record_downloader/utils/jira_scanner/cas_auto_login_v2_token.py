import requests
import json
import hashlib
import base64

# please install cryptography first
# python3 -m pip install cryptography
try:
    from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
    from cryptography.hazmat.backends import default_backend
except Exception as e:
    print("ERROR: cryptography not found!")
    print("Please install cryptography first, run command:")
    print("python3 -m pip install cryptography")
    exit(-1)
import configparser
from datetime import datetime
from  utils.jira_scanner.global_config import *


def _gen_gson_safe_chars_map():
    """
    static {
        REPLACEMENT_CHARS = new String[128];
        for (int i = 0; i <= 0x1f; i++) {
            REPLACEMENT_CHARS[i] = String.format("\\u%04x", (int) i);
        }
        REPLACEMENT_CHARS['"'] = "\\\"";
        REPLACEMENT_CHARS['\\'] = "\\\\";
        REPLACEMENT_CHARS['\t'] = "\\t";
        REPLACEMENT_CHARS['\b'] = "\\b";
        REPLACEMENT_CHARS['\n'] = "\\n";
        REPLACEMENT_CHARS['\r'] = "\\r";
        REPLACEMENT_CHARS['\f'] = "\\f";
        HTML_SAFE_REPLACEMENT_CHARS = REPLACEMENT_CHARS.clone();
        HTML_SAFE_REPLACEMENT_CHARS['<'] = "\\u003c";
        HTML_SAFE_REPLACEMENT_CHARS['>'] = "\\u003e";
        HTML_SAFE_REPLACEMENT_CHARS['&'] = "\\u0026";
        HTML_SAFE_REPLACEMENT_CHARS['='] = "\\u003d";
        HTML_SAFE_REPLACEMENT_CHARS['\''] = "\\u0027";
    }
    """

    # below characters in python3 will be automatic converted to safe characters
    # see json/encoder.py:21
    # + (0x0000...0x0001f, '\\u{:04x}')
    # + ('"', '\\\"'),
    # + ('\\', '\\\\'),
    # + ('\t', '\\t'),
    # + ('\b', '\\b'),
    # + ('\n', '\\n'),
    # + ('\r', '\\r'),
    # + ('\f', '\\f'),
    return (
        ("<", "\\u003c"),
        (">", "\\u003e"),
        ("&", "\\u0026"),
        ("=", "\\u003d"),
        ("'", "\\u0027"),
    )


_gson_safe_chars_map = _gen_gson_safe_chars_map()


def json_to_string(data):
    """
    一个兼容gson的字符串转换
    """
    string = json.dumps(data, separators=(",", ":"))
    for src, dest in _gson_safe_chars_map:
        string = string.replace(src, dest)
    return string


def x5_sign(appid, appkey, body):
    """
    计算x5头部的sign值
    """
    if not isinstance(body, str):
        body = json_to_string(body)

    concated = appid + body + appkey
    res = hashlib.md5(concated.encode(encoding="utf-8"))
    res = res.hexdigest()
    res = res.upper()
    return res


def x5_data(header, body):
    """
    生成x5协议POST的载荷
    """
    data = {
        "header": header,
        "body": body,
    }

    assert header.get("sign", None), "`sign` cannot be empty in x5 header"

    data = json_to_string(data)
    data = base64.b64encode(data.encode("utf-8"))
    data = data.decode("utf-8")
    return {"data": data}


def encrypt(appkey, content):
    """
    auto login的密码加密
    """

    # PKCS5Padding
    def add_paddings(block_size):
        padding = block_size - len(content) % block_size
        return content + chr(padding) * padding

    backend = default_backend()
    cipher = Cipher(
        algorithms.AES(appkey.encode("utf-8")), modes.ECB(), backend=backend
    )
    encryptor = cipher.encryptor()
    content = add_paddings(16)
    encrypted_data = encryptor.update(content.encode("utf-8"))
    res = base64.b64encode(encrypted_data)
    res = res.decode("utf-8")
    return res


def main_v1():
    body = {
        "username": username,
        "password": encrypt(appkey, password),
        "service": "https://jiratest.mioffice.cn/",
    }

    header = {
        "appid": appid,
        "sign": x5_sign(appid, appkey, body),
    }

    url = "https://cas.mioffice.cn/v2/api/auto/login"

    data = x5_data(header, body)

    resp = requests.post(url, data, allow_redirects=False)

    res = json.loads(resp.content.decode("utf-8"))

    if res.get("status", -1) != 0:
        print(resp.status_code, resp.reason, resp.content.decode("utf-8"))
        exit(res.get("status", -1))

    redirect_to = res.get("data", {}).get("redirect_to", None)
    print(redirect_to)


## 以下是新增的代码
## 主要是重载了requests.Session，为了隐藏cas白名单登录的细节
## 这个不用去操心米盾或者需要处理多次重定向的事情，只需要找到你访问服务的入口url就行。


class CASSession(requests.Session):
    """
    实现了自动登录的session封装
    """

    CAS_URL = "https://cas.mioffice.cn/login"

    CAS_AUTO_LOGIN_URL = "https://cas.mioffice.cn/v2/api/auto/login"

    # 以下几个变量，可以通过一个backend去存储，来扩展。
    # 为了演示，这里就简单地使用了变量。

    APP_ID = ""

    APP_KEY = ""

    USERNAME = ""

    PASSWORD = ""

    @classmethod
    def cas_config(cls, *, app_id, app_key, username, password):
        print(
            f"app_id: {app_id}, app_key: {app_key}, username: {username}, password: {password}"
        )
        cls.APP_ID = app_id
        cls.APP_KEY = app_key
        cls.USERNAME = username
        cls.PASSWORD = encrypt(app_key, password)

    @classmethod
    def cas_data(cls, service):
        body = {"username": cls.USERNAME, "password": cls.PASSWORD, "service": service}
        header = {"appid": cls.APP_ID, "sign": x5_sign(cls.APP_ID, cls.APP_KEY, body)}
        return x5_data(header, body)

    def cas_autologin(self, service):
        """
        调用cas的auto login接口。
        service为需要访问的url服务。
        如果成功，返回重定向的url；
        否则，返回None。
        """
        data = self.cas_data(service)
        res = self.post(self.CAS_AUTO_LOGIN_URL, data, allow_redirects=False)
        res = json.loads(res.content.decode("utf-8"))
        status = res.get("status", -1)
        if status != 0:
            print(f'auto login failed with {status} {res.get("message", "")}')
            return None
        return res.get("data", {}).get("redirect_to", None)

    def get_redirect_target(self, resp):
        """
        重载该方法是为了在重定向到cas登录页面的时候，调用cas的auto login获取自动的重定向的url。
        """
        import urllib.parse as urlparse

        if resp.is_redirect:
            location = resp.headers["location"]

            if location.startswith(self.CAS_URL):
                query = urlparse.urlparse(location).query
                url = urlparse.parse_qs(query).get("service", "")

                if not isinstance(url, str):
                    url = url[0]

                return self.cas_autologin(url)
            else:
                return super().get_redirect_target(resp)
        return None


## 以下是新增的代码 - 2022.02.15
## 重载equests.auth.AuthBase，重写__cal__方法，在requests的请求头中加入了Authorization


class HTTPBearerAuth(requests.auth.AuthBase):
    def __init__(self, ini_file):
        self.config = configparser.ConfigParser()
        self.ini_file = ini_file
        self.config.read(self.ini_file)
        if "DEFAULT" in self.config:
            self.token = self.config["DEFAULT"]["TOKEN"]
        else:
            self.token = ""

    def __eq__(self, other):
        return self.token == getattr(other, "token", None)

    def __ne__(self, other):
        return not self == other

    def __call__(self, r):
        r.headers["Authorization"] = "Bearer " + self.token
        return r


## 以下是新增的代码 - 2022.02.15
## 通过TokenManger来更新和维护Token的有效生命周期
class TokenManger(object):
    def __init__(self, session, domain, ini_file):
        self.session = session
        self.ini_file = ini_file
        self.url = domain if domain.startswith("https://") else "https://" + domain
        self.token_url = "{url}/rest/pat/latest/tokens".format(url=self.url)

    def list_tokens(self):
        resp = self.session.get(self.token_url, auth=HTTPBearerAuth(self.ini_file))
        return self.session.get(
            self.token_url, auth=HTTPBearerAuth(self.ini_file)
        ).json()

    def create_token(self, name):
        create_info = {"name": name, "expirationDuration": 90}
        headers = {"content-type": "application/json"}
        resp = self.session.post(
            self.token_url,
            data=json.dumps(create_info),
            headers=headers,
            auth=HTTPBearerAuth(self.ini_file),
        )
        return resp.json()

    def delete_token(self, id):
        detele_url = "{url}/{id}".format(url=self.token_url, id=id)
        headers = {"content-type": "application/json"}
        resp = self.session.delete(
            detele_url, auth=HTTPBearerAuth(self.ini_file), headers=headers
        )

    def save_token(self, token):
        config = configparser.ConfigParser()
        config["DEFAULT"]["TOKEN"] = token
        with open(self.ini_file, "w") as configfile:
            config.write(configfile)

    def auto_update(self, name="jiratoken", valid_date=30):
        """
        更新逻辑，name为jiratoken的，离过期时间等于30天，则自动更新。
        如果对应name的token不存在，则创建一个新token
        """
        all_tokens = self.list_tokens()
        print("[*] Old tokens: ", all_tokens)
        is_update = False
        is_exists = False
        for each_token in all_tokens:
            token_name = each_token.get("name", "")
            expiringAt = each_token.get("expiringAt")
            if name == token_name:
                is_exists = True
                # 判断过期时间与当前时间是否小于valid_date
                # expiring_date = datetime.fromisoformat(expiringAt).date()
                expiring_date = datetime.strptime(expiringAt[:10], "%Y-%m-%d").date()
                now_date = datetime.now().date()
                left_date = expiring_date - now_date
                if left_date.days <= valid_date:
                    is_update = True
                    break

        if is_update == True or is_exists == False:
            new_token = self.create_token(name)
            # print("[*] New token: ", new_token)
            if new_token.get("rawToken", "") != "":
                self.save_token(new_token.get("rawToken"))

        for each_token in all_tokens:
            token_name = each_token.get("name", "")
            if name == token_name and is_update == True:
                id = each_token.get("id")
                self.delete_token(id)
        all_tokens = self.list_tokens()
        # print("[*] Now tokens", all_tokens)


appid = APP_ID
appkey = APP_KEY
username = USER_NAME
password = PASSWORD


def create_session(url):
    # Step1. 使用requests.Session 来自动维护Cookie，主要用于获取米盾_aegis_cas的Cookie以及在后续请求中带上
    session = CASSession()

    session.cas_config(
        app_id=appid, app_key=appkey, username=username, password=password
    )

    resp = session.get(url)
    if resp.status_code != 200:
        print("login failed.")
        print(resp.content)
        exit(-1)

    # Step2. 复用session进行token管理
    token_manager = TokenManger(session, url, CONFIG_PATH)
    # 这里可以根据token有效时间进行更新
    token_manager.auto_update(name=TOKEN_NAME, valid_date=30)

    return session
