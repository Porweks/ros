#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from fullnamesumservice.srv import Fullnamesumservice

class MinimalService(Node):
    def __init__(self):
            super().__init__('minimal_service')
            self.srv = self.create_service(Fullnamesumservice, 'full_name', self.handle_summ_full_name)


    def handle_summ_full_name(self, request, response):
        # Склеиваем фамилию, имя и отчество в одну строку
        response.full_name = request.surname + " " + request.name + " " + request.patronymic
        return response

def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()