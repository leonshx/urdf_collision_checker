#!/usr/bin/env bash
# Project-local Linux setup. Does not require sudo or change shell profiles.
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEV=0
START=0
usage() {
    cat <<'EOF'
用法：./setup.sh [--dev] [--start]
  默认       安装 Python 3.12、运行依赖、Node.js 和 Xacro，构建网页
  --dev      同时安装 pytest、Playwright 等开发依赖
  --start    配置成功后启动服务（http://127.0.0.1:8765）
  -h, --help 显示帮助

要求 Linux、网络连接，以及 curl 或 wget（已有 uv 时不需要）。
不需要 sudo，不修改系统 Python 或 shell 配置；可重复运行。
--dev 不下载浏览器；浏览器测试另执行 .venv/bin/playwright install chromium。
EOF
}
for arg in "$@"; do
    case "$arg" in
        --dev) DEV=1 ;;
        --start) START=1 ;;
        -h|--help) usage; exit 0 ;;
        *) printf '未知参数：%s\n' "$arg" >&2; usage >&2; exit 2 ;;
    esac
done
if [[ "$(uname -s)" != Linux ]]; then
    printf '此脚本目前仅支持 Linux。\n' >&2
    exit 1
fi

INSTALLER=''
cleanup() { [[ -z "$INSTALLER" ]] || rm -f -- "$INSTALLER"; }
trap cleanup EXIT
trap 'printf "\n环境配置失败（第 %s 行），修复上述错误后可重新运行。\n" "$LINENO" >&2' ERR

mkdir -p "$ROOT/.tools"
if [[ -x "$ROOT/.tools/uv/uv" ]]; then
    UV="$ROOT/.tools/uv/uv"
elif command -v uv >/dev/null 2>&1; then
    UV="$(command -v uv)"
else
    printf '\n[1/4] 安装项目内 uv 0.13.8…\n'
    INSTALLER="$(mktemp)"
    if command -v curl >/dev/null 2>&1; then
        curl --fail --location --retry 3 https://astral.sh/uv/0.13.8/install.sh --output "$INSTALLER"
    elif command -v wget >/dev/null 2>&1; then
        wget -O "$INSTALLER" https://astral.sh/uv/0.13.8/install.sh
    else
        printf '请先安装 curl 或 wget，或自行安装 uv。\n' >&2
        exit 1
    fi
    env UV_UNMANAGED_INSTALL="$ROOT/.tools/uv" sh "$INSTALLER"
    UV="$ROOT/.tools/uv/uv"
fi

# Keep the environment in this checkout, even when invoked from another directory.
export UV_PROJECT_ENVIRONMENT="$ROOT/.venv"
export UV_PYTHON_INSTALL_DIR="${UV_PYTHON_INSTALL_DIR:-$ROOT/.tools/python}"
export UV_CACHE_DIR="${UV_CACHE_DIR:-$ROOT/.tools/cache/uv}"
export npm_config_cache="${npm_config_cache:-$ROOT/.tools/cache/npm}"
export PATH="$ROOT/.venv/bin:$PATH"
cd "$ROOT"
printf '\n[1/4] 使用 %s\n' "$("$UV" --version)"
printf '\n[2/4] 安装锁定的 Python、Node.js 和 Xacro 依赖…\n'
EXTRAS=(--extra setup --extra xacro)
if (( DEV )); then EXTRAS+=(--extra dev); fi
"$UV" sync --locked --python 3.12 "${EXTRAS[@]}"

printf '\n[3/4] 安装前端依赖并构建网页…\n'
npm --prefix "$ROOT/frontend" ci --no-audit --no-fund
npm --prefix "$ROOT/frontend" run build

printf '\n[4/4] 验证运行依赖…\n'
"$ROOT/.venv/bin/python" - <<'PY'
import importlib.metadata
import coal
import fastapi
import pinocchio
import xacro

for name in ("pin", "coal", "fastapi", "xacro", "nodejs-wheel"):
    print(f"{name}: {importlib.metadata.version(name)}")
PY
node --version
printf '\n配置完成。启动命令：\n  '
printf '%q ' "$ROOT/.venv/bin/urdf-collision-checker"
printf '\n打开 http://127.0.0.1:8765\n'
if (( START )); then
    exec "$ROOT/.venv/bin/urdf-collision-checker"
fi
