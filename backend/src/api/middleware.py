from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response as StarletteResponse
from typing import Callable, Awaitable
import time
import logging
from src.utils.logger import setup_logger


class RequestLoggingMiddleware(BaseHTTPMiddleware):
    """
    Middleware to log incoming requests and outgoing responses.
    """

    def __init__(self, app, logger_name: str = "api"):
        super().__init__(app)
        self.logger = setup_logger(logger_name)

    async def dispatch(
        self,
        request: Request,
        call_next: Callable[[Request], Awaitable[Response]]
    ) -> StarletteResponse:
        # Log request
        start_time = time.time()
        self.logger.info(
            f"Request: {request.method} {request.url.path} "
            f"from {request.client.host if request.client else 'unknown'}"
        )

        # Process request
        response = await call_next(request)

        # Calculate processing time
        process_time = time.time() - start_time

        # Log response
        self.logger.info(
            f"Response: {response.status_code} in {process_time:.2f}ms "
            f"for {request.method} {request.url.path}"
        )

        return response


class MetricsMiddleware(BaseHTTPMiddleware):
    """
    Middleware to collect basic metrics about requests.
    """

    def __init__(self, app):
        super().__init__(app)
        self.request_count = 0
        self.error_count = 0
        self.total_response_time = 0.0
        self.logger = setup_logger("metrics")

    async def dispatch(
        self,
        request: Request,
        call_next: Callable[[Request], Awaitable[Response]]
    ) -> StarletteResponse:
        # Increment request count
        self.request_count += 1
        start_time = time.time()

        try:
            response = await call_next(request)
        except Exception as e:
            self.error_count += 1
            raise e
        finally:
            # Calculate and store response time
            process_time = time.time() - start_time
            self.total_response_time += process_time

            # Log metrics periodically
            if self.request_count % 100 == 0:
                avg_response_time = self.total_response_time / self.request_count
                self.logger.info(
                    f"Metrics - Requests: {self.request_count}, "
                    f"Errors: {self.error_count}, "
                    f"Avg Response Time: {avg_response_time:.2f}s"
                )

        return response