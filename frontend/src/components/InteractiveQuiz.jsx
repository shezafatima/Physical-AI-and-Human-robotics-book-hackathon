import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './InteractiveQuiz.module.css';
import { submitQuizAnswers } from '../services/api';

const InteractiveQuiz = ({ quizData, className }) => {
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [selectedAnswers, setSelectedAnswers] = useState({});
  const [showResults, setShowResults] = useState(false);
  const [score, setScore] = useState(0);

  const handleAnswerSelect = (questionIndex, answerIndex) => {
    if (showResults) return;

    const newSelectedAnswers = {
      ...selectedAnswers,
      [questionIndex]: answerIndex
    };
    setSelectedAnswers(newSelectedAnswers);
  };

  const handleNextQuestion = () => {
    if (currentQuestion < quizData.questions.length - 1) {
      setCurrentQuestion(currentQuestion + 1);
    }
  };

  const handlePreviousQuestion = () => {
    if (currentQuestion > 0) {
      setCurrentQuestion(currentQuestion - 1);
    }
  };

  const calculateScore = async () => {
    try {
      // Submit answers to backend for processing
      const result = await submitQuizAnswers(quizData.id, selectedAnswers);

      // If backend returns the score, use it; otherwise, calculate locally
      let correctAnswers = 0;
      if (result && result.score !== undefined) {
        setScore(result.score);
      } else {
        quizData.questions.forEach((question, index) => {
          if (selectedAnswers[index] !== undefined && question.correctAnswer === selectedAnswers[index]) {
            correctAnswers++;
          }
        });
        setScore(correctAnswers);
      }

      setShowResults(true);
    } catch (error) {
      console.error('Error submitting quiz:', error);

      // Fallback to local calculation if API fails
      let correctAnswers = 0;
      quizData.questions.forEach((question, index) => {
        if (selectedAnswers[index] !== undefined && question.correctAnswer === selectedAnswers[index]) {
          correctAnswers++;
        }
      });
      setScore(correctAnswers);
      setShowResults(true);
    }
  };

  const resetQuiz = () => {
    setCurrentQuestion(0);
    setSelectedAnswers({});
    setShowResults(false);
    setScore(0);
  };

  const currentQ = quizData.questions[currentQuestion];
  const selectedAnswer = selectedAnswers[currentQuestion];

  return (
    <div className={clsx(styles.quizContainer, className)}>
      <div className={styles.quizHeader}>
        <h3>{quizData.title}</h3>
        <p className={styles.quizDescription}>{quizData.description}</p>
      </div>

      {!showResults ? (
        <div className={styles.quizContent}>
          <div className={styles.questionCounter}>
            Question {currentQuestion + 1} of {quizData.questions.length}
          </div>

          <div className={styles.question}>
            <h4>{currentQ.question}</h4>
            <div className={styles.answers}>
              {currentQ.options.map((option, index) => (
                <button
                  key={index}
                  className={clsx(
                    styles.answerOption,
                    selectedAnswer === index && styles.selected,
                    showResults && index === currentQ.correctAnswer && styles.correct,
                    showResults && selectedAnswer === index && selectedAnswer !== currentQ.correctAnswer && styles.incorrect
                  )}
                  onClick={() => handleAnswerSelect(currentQuestion, index)}
                >
                  <span className={styles.optionLetter}>
                    {String.fromCharCode(65 + index)}.
                  </span>
                  <span className={styles.optionText}>{option}</span>
                  {showResults && index === currentQ.correctAnswer && (
                    <span className={styles.correctIndicator}>✓</span>
                  )}
                  {showResults && selectedAnswer === index && selectedAnswer !== currentQ.correctAnswer && (
                    <span className={styles.incorrectIndicator}>✗</span>
                  )}
                </button>
              ))}
            </div>
          </div>

          <div className={styles.navigation}>
            <button
              onClick={handlePreviousQuestion}
              disabled={currentQuestion === 0}
              className={styles.navButton}
            >
              Previous
            </button>

            {currentQuestion < quizData.questions.length - 1 ? (
              <button
                onClick={handleNextQuestion}
                disabled={selectedAnswer === undefined}
                className={clsx(styles.navButton, styles.primary)}
              >
                Next
              </button>
            ) : (
              <button
                onClick={calculateScore}
                disabled={selectedAnswer === undefined}
                className={clsx(styles.navButton, styles.primary)}
              >
                Submit Quiz
              </button>
            )}
          </div>
        </div>
      ) : (
        <div className={styles.results}>
          <div className={styles.scoreCard}>
            <h4>Quiz Results</h4>
            <div className={styles.score}>
              <span className={styles.scoreNumber}>{score}</span>
              <span className={styles.scoreTotal}>/{quizData.questions.length}</span>
            </div>
            <p className={styles.scorePercentage}>
              {Math.round((score / quizData.questions.length) * 100)}% Correct
            </p>
          </div>

          <div className={styles.reviewSection}>
            <h5>Review Your Answers</h5>
            {quizData.questions.map((question, qIndex) => (
              <div key={qIndex} className={styles.questionReview}>
                <div className={clsx(
                  styles.reviewQuestion,
                  selectedAnswers[qIndex] === question.correctAnswer ? styles.correct : styles.incorrect
                )}>
                  <strong>Q{qIndex + 1}:</strong> {question.question}
                </div>
                <div className={styles.reviewAnswer}>
                  <div className={styles.userAnswer}>
                    <strong>Your answer:</strong> {question.options[selectedAnswers[qIndex]]}
                    {selectedAnswers[qIndex] !== undefined && selectedAnswers[qIndex] !== question.correctAnswer && (
                      <span className={styles.wrong}> (Incorrect)</span>
                    )}
                    {selectedAnswers[qIndex] !== undefined && selectedAnswers[qIndex] === question.correctAnswer && (
                      <span className={styles.correct}> (Correct)</span>
                    )}
                  </div>
                  {selectedAnswers[qIndex] !== question.correctAnswer && (
                    <div className={styles.correctAnswer}>
                      <strong>Correct answer:</strong> {question.options[question.correctAnswer]}
                    </div>
                  )}
                </div>
              </div>
            ))}
          </div>

          <button onClick={resetQuiz} className={clsx(styles.navButton, styles.primary)}>
            Retake Quiz
          </button>
        </div>
      )}
    </div>
  );
};

// Example usage data
InteractiveQuiz.defaultProps = {
  quizData: {
    title: "Physical AI Fundamentals Quiz",
    description: "Test your knowledge of Physical AI and Humanoid Robotics concepts",
    questions: [
      {
        question: "What is the primary goal of Physical AI in robotics?",
        options: [
          "To create robots that can think like humans",
          "To develop robots that can interact with the physical world effectively",
          "To build robots that can replace humans completely",
          "To create robots that can only work in controlled environments"
        ],
        correctAnswer: 1
      },
      {
        question: "Which of the following is a key challenge in humanoid robotics?",
        options: [
          "Designing aesthetically pleasing robots",
          "Achieving stable bipedal locomotion",
          "Creating robots with multiple arms",
          "Building robots that are very large"
        ],
        correctAnswer: 1
      },
      {
        question: "What does RAG stand for in the context of AI systems?",
        options: [
          "Robotic Action Generation",
          "Retrieval-Augmented Generation",
          "Robotics and Automation Group",
          "Real-time Adaptive Guidance"
        ],
        correctAnswer: 1
      }
    ]
  }
};

export default InteractiveQuiz;