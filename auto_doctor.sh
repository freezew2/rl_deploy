#!/usr/bin/env bash

set -e

containers=($(docker ps -a --format "{{.Names}}"))

if [ ${#containers[@]} -eq 0 ]; then
  echo "❌ 当前没有任何 Docker 容器"
  exit 1
fi

echo "📦 可用的 Docker 容器："
for i in "${!containers[@]}"; do
  status=$(docker inspect -f '{{.State.Status}}' "${containers[$i]}")
  printf "  [%d] %-30s (%s)\n" "$i" "${containers[$i]}" "$status"
done

echo
read -p "👉 请输入要进入的容器编号: " idx

if ! [[ "$idx" =~ ^[0-9]+$ ]] || [ "$idx" -ge "${#containers[@]}" ]; then
  echo "❌ 无效编号"
  exit 1
fi

container="${containers[$idx]}"

status=$(docker inspect -f '{{.State.Status}}' "$container")
if [ "$status" != "running" ]; then
  echo "▶️ 容器未运行，正在启动 $container ..."
  docker start "$container" > /dev/null
fi

echo
read -p "是否以 root 用户进入？[y/N]: " use_root

if [[ "$use_root" == "y" || "$use_root" == "Y" ]]; then
  echo "🔑 以 root 进入容器 $container"
  docker exec -it -u root "$container" /bin/bash
else
  echo "👤 以普通用户进入容器 $container"
  docker exec -it "$container" /bin/bash
fi

