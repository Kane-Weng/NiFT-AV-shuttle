"""
python-dotenv stand-in (ARCH section 5): `dotenv_values` and `load_dotenv` for `octane_client.octane_config()`.

It parses `KEY=VALUE` lines the way python-dotenv does for the simple cases: blank lines and `#` comments are skipped, an `export ` prefix is dropped, single or double quotes around a value are removed, and an unquoted value ends at ` #`. A key without `=` maps to None. Variable expansion is not done. In the playground the launch set puts every MCITY_OCTANE_* key in os.environ, which wins over a .env file, and the bundle never ships a .env.
"""

import io
import os


def _parse(text):
    out = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        if line.startswith('export '):
            line = line[len('export '):].lstrip()
        if '=' not in line:
            out[line] = None
            continue
        key, value = line.split('=', 1)
        key, value = key.strip(), value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ('"', "'"):
            quote, value = value[0], value[1:-1]
            if quote == '"':
                value = value.replace('\\n', '\n').replace('\\"', '"')
        elif ' #' in value:
            value = value.split(' #', 1)[0].rstrip()
        out[key] = value
    return out


def dotenv_values(dotenv_path=None, stream=None, verbose=False, interpolate=True, encoding='utf-8'):
    """Return {key: value} from a .env file or a text stream; {} for None or a missing file."""
    if stream is not None:
        return _parse(stream.read() if hasattr(stream, 'read') else str(stream))
    if dotenv_path is None:
        return {}
    try:
        with io.open(os.fspath(dotenv_path), encoding=encoding) as f:
            return _parse(f.read())
    except OSError:
        return {}


def load_dotenv(dotenv_path=None, stream=None, verbose=False, override=False, interpolate=True,
                encoding='utf-8'):
    """Put the values into os.environ (existing ones win unless override). True if anything was read."""
    values = dotenv_values(dotenv_path, stream, verbose, interpolate, encoding)
    for k, v in values.items():
        if v is not None and (override or k not in os.environ):
            os.environ[k] = v
    return bool(values)


def find_dotenv(filename='.env', raise_error_if_not_found=False, usecwd=False):
    """Not searched in the playground: always ''."""
    if raise_error_if_not_found:
        raise IOError('File not found')
    return ''
