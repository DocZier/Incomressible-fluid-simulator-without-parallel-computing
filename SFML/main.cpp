#include "Fluid.h"
#include <iostream>
#include <SFML/Graphics.hpp>
#include<chrono>

int main()
{
	//Параметры поля
	//Размеры поля
	int N = 200;
	int M = 200;
	//Размер точки
	int Size = 4;

	//Параметры жидкости
	double diffusion = 0.5;
	double visconcy = 0.5;

	//Параметры симуляции
	int framerate = 60;
	double dt = 1.0 / framerate;
	int Iterations = 10;

	//Положение курсора
	int cursor_x = 0;
	int cursor_y = 0;
	int cursor_xprev = 0;
	int cursor_yprev = 0;

	sf::RenderWindow window(sf::VideoMode((N + 2) * Size, (M + 2) * Size), "2D Incomressible fluid simulation");
	window.setFramerateLimit(framerate);
	sf::Image image;
	sf::Texture texture;
	sf::Sprite sprite;
	image.create((N + 2) * Size, (M + 2) * Size, sf::Color::Black);

	Fluid fluid(N, M, diffusion, visconcy, dt, Iterations);

	sf::Clock clock;
	while (window.isOpen())
	{
		sf::Event event;
		double current = clock.restart().asSeconds();
		double fps = 1.0f / (current);
		while (window.pollEvent(event))
		{

			if (event.type == sf::Event::Closed)
				window.close();
			//Завершение работы
			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape)
				window.close();
			//Сброс
			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::R)
				fluid.Reset();
		}
		cursor_xprev = cursor_x;
		cursor_yprev = cursor_y;
		sf::Vector2i localPosition = sf::Mouse::getPosition(window);
		cursor_x = localPosition.x;
		cursor_y = localPosition.y;

		//Взаимодействие пользователя
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
		{
			if ((cursor_x / Size > 0 && cursor_x / Size < N - 1 && cursor_y / Size  > 0 && cursor_y / Size < M - 1) && (cursor_xprev / Size > 0 && cursor_xprev / Size < N - 1 && cursor_yprev / Size  > 0 && cursor_yprev / Size < M - 1))
			{
				fluid.ApplyForce(cursor_xprev / Size, cursor_yprev / Size, cursor_x / Size, cursor_y / Size);
			}
		}
		fluid.Update();
		fluid.Render(Size, image);
		texture.loadFromImage(image);
		sprite.setTexture(texture, true);
		window.draw(sprite);
		window.display();

		std::cout << fps << '\n';
	}
	return 0;
}