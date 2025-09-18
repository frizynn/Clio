#!/bin/bash

echo "🐳 Configurando Docker para Clio..."

# Crear directorios necesarios
mkdir -p datasets
mkdir -p clio_workspace

# Verificar si Docker está instalado
if ! command -v docker &> /dev/null; then
    echo "❌ Docker no está instalado. Instálalo desde: https://www.docker.com/products/docker-desktop"
    exit 1
fi

# Verificar si docker-compose está instalado
if ! command -v docker-compose &> /dev/null; then
    echo "❌ docker-compose no está instalado. Instálalo desde: https://docs.docker.com/compose/install/"
    exit 1
fi

echo "✅ Docker y docker-compose encontrados"

# Construir la imagen
echo "🔨 Construyendo imagen Docker..."
docker-compose build

if [ $? -eq 0 ]; then
    echo "✅ Imagen construida exitosamente"
else
    echo "❌ Error al construir la imagen"
    exit 1
fi

# Crear archivos de ejemplo
echo "📁 Creando archivos de ejemplo..."

# Crear archivo de tareas de ejemplo
cat > datasets/tasks.yaml << EOF
bring me a pillow: []
clean toaster: []
find deck of cards: []
read brown textbook: []
find red cup: []
EOF

# Crear archivo de tareas de lugares de ejemplo
cat > datasets/region_tasks.yaml << EOF
kitchen: []
workspace: []
living room: []
bedroom: []
EOF

echo "✅ Archivos de ejemplo creados en ./datasets/"

echo ""
echo "🎉 ¡Configuración completada!"
echo ""
echo "📋 Comandos disponibles:"
echo "  docker-compose up -d          # Iniciar en segundo plano"
echo "  docker-compose up             # Iniciar en primer plano"
echo "  docker-compose down           # Detener"
echo "  docker-compose exec clio bash # Entrar al contenedor"
echo "  docker-compose logs -f        # Ver logs"
echo ""
echo "🚀 Para iniciar Clio:"
echo "  docker-compose up"
echo ""
echo "📁 Datasets:"
echo "  Coloca tus archivos .bag en ./datasets/"
echo "  Los archivos de tareas ya están creados"
