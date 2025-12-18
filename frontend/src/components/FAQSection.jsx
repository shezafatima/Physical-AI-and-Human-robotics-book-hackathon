import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './FAQSection.module.css';

const FAQSection = () => {
  const [openIndex, setOpenIndex] = useState(null);

  const toggleFAQ = (index) => {
    setOpenIndex(openIndex === index ? null : index);
  };

  const faqs = [
    {
      question: "What is Physical AI?",
      answer: "Physical AI is an approach that integrates artificial intelligence with physical systems, focusing on embodied intelligence where AI agents interact with and learn from the physical world through sensors and actuators."
    },
    {
      question: "Do I need prior robotics experience?",
      answer: "No prior robotics experience is required. This coursebook starts with fundamentals and gradually builds up to advanced topics, making it accessible for beginners."
    },
    {
      question: "What programming languages are used?",
      answer: "The course primarily uses Python for ROS 2 development, C++ for performance-critical applications, and JavaScript for web-based interactive elements."
    },
    {
      question: "How does the AI chatbot work?",
      answer: "Our RAG (Retrieval-Augmented Generation) chatbot uses vector embeddings to search through course materials and provide accurate answers based on the content you're studying."
    },
    {
      question: "Can I access this course on mobile devices?",
      answer: "Yes, the coursebook is fully responsive and works on all devices, including smartphones, tablets, and desktop computers."
    }
  ];

  return (
    <section className={styles.faqSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h2>Frequently Asked Questions</h2>
            <div className={styles.faqContainer}>
              {faqs.map((faq, index) => (
                <div key={index} className={styles.faqItem}>
                  <button
                    className={styles.faqQuestion}
                    onClick={() => toggleFAQ(index)}
                    aria-expanded={openIndex === index}
                  >
                    <span>{faq.question}</span>
                    <span className={styles.faqIcon}>{openIndex === index ? '−' : '+'}</span>
                  </button>
                  {openIndex === index && (
                    <div className={styles.faqAnswer}>
                      <p>{faq.answer}</p>
                    </div>
                  )}
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default FAQSection;