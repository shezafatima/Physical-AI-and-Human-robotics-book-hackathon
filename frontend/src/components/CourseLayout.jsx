// import React, { useState } from 'react';
// import clsx from 'clsx';
// import styles from './CourseLayout.module.css';
// import Chatbot from './Chatbot';

// const CourseLayout = ({ children, title, description, showChatbot = false, className, isHomePage = false }) => {
//   const [isChatbotOpen, setIsChatbotOpen] = useState(false); // Changed to default closed

//   const toggleChatbot = () => {
//     setIsChatbotOpen(!isChatbotOpen);
//   };

//   return (
//     <>
//       {/* Content header only */}
//       <header className={styles.contentHeader}>
//         <div className={styles.headerContent}>
//           <div className={styles.headerText}>
//             <h1>{title}</h1>
//             {description && <p>{description}</p>}
//           </div>
//         </div>
//       </header>

//       {/* Main content area with floating chatbot */}
//       <div className={clsx(styles.contentContainer, isHomePage && styles.contentContainerHome)}>
//         <main className={clsx(styles.contentMain, isHomePage && styles.contentMainHome)}>
//           {children}
//         </main>

//         {/* Floating Chatbot */}
//         {showChatbot && (
//           <div className={clsx(styles.chatbotContainer, isChatbotOpen && styles.chatbotOpen)}>
//             <button
//               className={styles.chatbotToggle}
//               onClick={toggleChatbot}
//               aria-label={isChatbotOpen ? "Close chatbot" : "Open chatbot"}
//             >
//               {isChatbotOpen ? '×' : '💬'}
//             </button>
//             {isChatbotOpen && <Chatbot onClose={toggleChatbot} />}
//           </div>
//         )}
//       </div>
//     </>
//   );
// };

// export default CourseLayout;
import React from 'react';
import clsx from 'clsx';
import styles from './CourseLayout.module.css';

const CourseLayout = ({ children, title, description, className, isHomePage }) => {
  return (
    <>
      <header className={styles.contentHeader}>
        <div className={styles.headerContent}>
          <div className={styles.headerText}>
            <h1>{title}</h1>
            {description && <p>{description}</p>}
          </div>
        </div>
      </header>

      <div className={clsx(styles.contentContainer, isHomePage && styles.contentContainerHome)}>
        <main className={clsx(styles.contentMain, isHomePage && styles.contentMainHome)}>
          {children}
        </main>
      </div>
    </>
  );

};

export default CourseLayout;
