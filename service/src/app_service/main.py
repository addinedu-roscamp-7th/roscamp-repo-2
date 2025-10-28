from fastapi import FastAPI
from routers import cafe, informationDesk, dbtest, robot_control_system

app = FastAPI()

# 반드시 router 객체를 전달
app.include_router(cafe.router)
app.include_router(informationDesk.router)
app.include_router(dbtest.router)
app.include_router(robot_control_system.router)