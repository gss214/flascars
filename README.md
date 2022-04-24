# flascars

Trabalho final da matéria de Projeto e análise de algoritmos, com o professor Jan Mendonça. Universidade de Brasília (UnB), semestre 2021/2.

O trabalho se resume em um site que simula as funcionalidades de um Uber, ou algum aplicativo similar de viagens. É baseado em um mapa, clientes que estão em um lugar e querem ir para outro lugar, e carros que estão disponíveis para realizar essa viagem. O mapa é um grafo, portanto, todos os algoritmos são baseados em grafo.

O projeto foi feito em Flask, e as funcionalidades de grafo foram implementadas utiliando a biblioteca NetworkX com suas estruturas de dados. A biblioteca PyVis foi ultilizada para mostrar o grafo de maneira visual ao usuário.

## Como utilizar

Instale o **poetry**:

Windows (powershell)

```
(Invoke-WebRequest -Uri https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py -UseBasicParsing).Content | python -
```

Linux

```
curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python -
```

Reinicie o Terminal

Vá para a pasta raiz do projeto. Execute:
```
poetry install
```
Isso vai instalar todas as dependências que o projeto atualmente necessita. Algumas vezes, depois de um pull, se alguma dependência tiver mudado, pode ser que você precise rodar esse comando novamente

Use o seguinte comando para abrir um shell com o ambiente virtual criado:
```
poetry shell
```

E, por último, rode o site:
```
python run.py
```